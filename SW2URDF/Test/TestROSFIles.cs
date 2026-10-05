
using System;
using System.IO;
using System.Text;
using System.Xml;
using SW2URDF.ROS;
using SW2URDF.URDF;
using Xunit;

namespace SW2URDF.Test
{
    public class TestROSFIles
    {
        private static XmlDocument ReadXml(string xml)
        {
            var document = new XmlDocument { XmlResolver = null };
            var settings = new XmlReaderSettings { DtdProcessing = DtdProcessing.Prohibit };
            using (var reader = XmlReader.Create(new StringReader(xml), settings)) document.Load(reader);
            return document;
        }

        private static string Serialize(Robot robot, bool? useXacro)
        {
            var text = new StringBuilder();
            using (var writer = XmlWriter.Create(text))
            {
                if (useXacro.HasValue) robot.WriteURDF(writer, useXacro.Value);
                else robot.WriteURDF(writer);
            }
            return text.ToString();
        }

        [Fact]
        public void XacroAddsOnlyNamespaceAndDoesNotChangeLaterUrdfExports()
        {
            var robot = new Robot { Name = "test_robot" };
            robot.BaseLink.Name = "base_link";
            var child = new Link(robot.BaseLink) { Name = "child_link" };
            child.Joint.Name = "fixed_joint";
            child.Joint.Type = "fixed";
            child.Joint.Parent.Name = "base_link";
            child.Joint.Child.Name = "child_link";
            child.Joint.Origin.X = 0.125;
            robot.BaseLink.Children.Add(child);
            string before = Serialize(robot, null);
            var urdf = ReadXml(before);
            var xacro = ReadXml(Serialize(robot, true));
            Assert.Equal("http://www.ros.org/wiki/xacro", xacro.DocumentElement.GetAttribute("xmlns:xacro"));
            Assert.Equal("", xacro.DocumentElement.NamespaceURI);
            Assert.Equal(2, xacro.SelectNodes("/robot/link").Count);
            Assert.Equal(1, xacro.SelectNodes("/robot/joint").Count);
            xacro.DocumentElement.RemoveAttribute("xmlns:xacro");
            Assert.Equal(urdf.DocumentElement.OuterXml, xacro.DocumentElement.OuterXml);
            Assert.Equal(before, Serialize(robot, false));
            Assert.Equal(before, Serialize(robot, null));
        }

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void LaunchAndManifestUseSelectedFormat(bool useXacro)
        {
            string directory = Path.Combine(Path.GetTempPath(), Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(directory);
            try
            {
                string name = "test_robot" + (useXacro ? ".xacro" : ".urdf");
                string path = "$(find test_package)/urdf/" + name;
                new Rviz("test_package", name, useXacro).WriteFiles(directory + Path.DirectorySeparatorChar);
                new Gazebo("test_robot", "test_package", name, useXacro).WriteFile(directory + Path.DirectorySeparatorChar);
                new PackageXML("test_package", useXacro).WriteElement(new PackageXMLWriter(Path.Combine(directory, "package.xml")));
                var display = ReadXml(File.ReadAllText(Path.Combine(directory, "display.launch")));
                var gazebo = ReadXml(File.ReadAllText(Path.Combine(directory, "gazebo.launch")));
                var package = ReadXml(File.ReadAllText(Path.Combine(directory, "package.xml")));
                var parameter = (XmlElement)display.SelectSingleNode("/launch/param[@name='robot_description']");
                var spawn = (XmlElement)gazebo.SelectSingleNode("/launch/node[@name='spawn_model']");
                if (useXacro)
                {
                    Assert.Equal("$(find xacro)/xacro '" + path + "'", parameter.GetAttribute("command"));
                    Assert.False(parameter.HasAttribute("textfile"));
                    Assert.Equal(parameter.OuterXml, gazebo.SelectSingleNode("/launch/param[@name='robot_description']").OuterXml);
                    Assert.Contains("-param robot_description", spawn.GetAttribute("args"));
                    Assert.DoesNotContain("-file", spawn.GetAttribute("args"));
                    Assert.NotNull(package.SelectSingleNode("/package/exec_depend[text()='xacro']"));
                }
                else
                {
                    Assert.Equal(path, parameter.GetAttribute("textfile"));
                    Assert.False(parameter.HasAttribute("command"));
                    Assert.Null(gazebo.SelectSingleNode("/launch/param[@name='robot_description']"));
                    Assert.Contains("-file " + path, spawn.GetAttribute("args"));
                    Assert.Null(package.SelectSingleNode("/package/exec_depend[text()='xacro']"));
                }
            }
            finally { Directory.Delete(directory, true); }
        }

        [Fact]
        public void RobotFormatSelectionIsIndependentOfMeshFormat()
        {
            var app = new Moq.Mock<SolidWorks.Interop.sldworks.SldWorks>();
            var attribute = new Moq.Mock<SolidWorks.Interop.sldworks.AttributeDef>();
            app.Setup(sw => sw.DefineAttribute(Moq.It.IsAny<string>())).Returns(attribute.Object);
            using (var form = new UI.AssemblyExportForm(app.Object, new LinkNode(), null))
            {
                var urdf = (System.Windows.Forms.RadioButton)form.Controls.Find("radioButtonUrdf", true)[0];
                var xacro = (System.Windows.Forms.RadioButton)form.Controls.Find("radioButtonXacro", true)[0];
                var stl = (System.Windows.Forms.RadioButton)form.Controls.Find("radioButtonStl", true)[0];
                var color = (System.Windows.Forms.RadioButton)form.Controls.Find("radioButton3dxml", true)[0];
                Assert.True(urdf.Checked);
                Assert.True(stl.Checked);
                xacro.Checked = true;
                Assert.False(urdf.Checked);
                Assert.True(stl.Checked);
                Assert.Equal("Export Xacro Only...", form.Controls.Find("buttonLinksExportUrdfOnly", true)[0].Text);
                color.Checked = true;
                Assert.True(xacro.Checked);
                Assert.False(stl.Checked);
                urdf.Checked = true;
                Assert.True(color.Checked);
                Assert.False(xacro.Checked);
                Assert.Equal("Export URDF and Meshes...", form.Controls.Find("buttonLinksFinish", true)[0].Text);
            }
        }
    }
}
