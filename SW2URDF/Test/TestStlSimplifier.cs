using g3;
using Moq;
using SolidWorks.Interop.sldworks;
using SW2URDF.UI;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using System;
using System.IO;
using System.Linq;
using System.Runtime.Serialization;
using System.Windows.Forms;
using Xunit;

namespace SW2URDF.Test
{
    public class TestStlSimplifier
    {
        private delegate bool SaveAsResult(string name, int version, int options, object data, ref int errors, ref int warnings);

        [Theory]
        [InlineData(false)]
        [InlineData(true)]
        public void StlExportAppliesLimitOnlyWhenSelectedAndPreservesInertial(bool limit)
        {
            var helper = (ExportHelper)System.Runtime.Serialization.FormatterServices.GetUninitializedObject(typeof(ExportHelper));
            var app = new Mock<ISldWorks>(); var document = new Mock<ModelDoc2>();
            var extension = new Mock<ModelDocExtension>(); var selection = new Mock<SelectionMgr>();
            selection.Setup(x => x.CreateSelectData()).Returns(new Mock<SelectData>().Object);
            document.SetupGet(x => x.SelectionManager).Returns(selection.Object);
            document.SetupGet(x => x.Extension).Returns(extension.Object);
            int errors = 0, warnings = 0;
            var mesh = Sphere(16);
            extension.Setup(x => x.SaveAs(It.IsAny<string>(), It.IsAny<int>(), It.IsAny<int>(), It.IsAny<object>(), ref errors, ref warnings))
                .Returns(new SaveAsResult((string name, int version, int options, object data, ref int e, ref int w) => { Write(name, mesh); return true; }));
            helper.iSwApp = app.Object; helper.ActiveSWModel = document.Object;
            var link = new Link { Name = "test", STLUseFaceLimit = limit, STLMaximumFaces = 500 };
            link.Joint.CoordinateSystemName = "frame";
            link.Inertial.Mass.Value = 3;
            link.Inertial.Origin.SetXYZ(new double[] { 0.1, 0.2, 0.3 });
            link.Inertial.Inertia.SetMomentMatrix(new double[] { 1, 0.1, 0.2, 0.1, 2, 0.3, 0.2, 0.3, 3 });
            double[] inertia = link.Inertial.Inertia.GetMoment();
            string path = Path.GetTempFileName();
            try
            {
                new Microsoft.VisualStudio.TestTools.UnitTesting.PrivateObject(helper).Invoke("SaveSTL", link, path);
                int count = StlSimplifier.ReadTriangleCount(path);
                if (limit) Assert.InRange(count, 4, 500);
                else Assert.Equal(mesh.TriangleCount, count);
                Assert.Equal(3, link.Inertial.Mass.Value);
                Assert.Equal(new double[] { 0.1, 0.2, 0.3 }, link.Inertial.Origin.GetXYZ());
                Assert.Equal(inertia, link.Inertial.Inertia.GetMoment());
            }
            finally { File.Delete(path); }
        }

        private static void Write(string path, DMesh3 mesh)
        {
            using (var writer = new BinaryWriter(File.Create(path)))
            {
                writer.Write(new byte[80]); writer.Write((uint)mesh.TriangleCount);
                foreach (int id in mesh.TriangleIndices())
                {
                    writer.Write(new byte[12]);
                    var triangle = mesh.GetTriangle(id);
                    foreach (int vertex in new[] { triangle.a, triangle.b, triangle.c })
                    {
                        var p = mesh.GetVertex(vertex);
                        writer.Write((float)p.x); writer.Write((float)p.y); writer.Write((float)p.z);
                    }
                    writer.Write((ushort)0);
                }
            }
        }

        private static DMesh3 Sphere(int edgeVertices)
        {
            var generator = new Sphere3Generator_NormalizedCube { Radius = 0.1, EdgeVertices = edgeVertices, NoSharedVertices = false };
            generator.Generate();
            var mesh = generator.MakeDMesh();
            foreach (int id in mesh.VertexIndices()) mesh.SetVertex(id, mesh.GetVertex(id) + new Vector3d(1, 2, 3));
            return mesh;
        }

        [Theory]
        [InlineData(16, 500)]
        [InlineData(131, 199999)]
        public void QemReachesLimitWithoutChangingUnitsOrOrigin(int edgeVertices, int maximum)
        {
            string path = Path.GetTempFileName();
            try
            {
                Write(path, Sphere(edgeVertices));
                Assert.True(StlSimplifier.ReadTriangleCount(path) > maximum);
                int count = StlSimplifier.LimitFaces(path, maximum);
                Assert.InRange(count, 4, maximum);
                Assert.Equal(84L + count * 50L, new FileInfo(path).Length);
                using (var reader = new BinaryReader(File.OpenRead(path)))
                {
                    reader.BaseStream.Position = 84;
                    double volume = 0;
                    for (int i = 0; i < count; i++)
                    {
                        var normal = new Vector3d(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                        var p = new Vector3d[3];
                        for (int j = 0; j < 3; j++)
                        {
                            p[j] = new Vector3d(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle()) - new Vector3d(1, 2, 3);
                            Assert.InRange(p[j].Length, 0.095, 0.105);
                        }
                        reader.ReadUInt16();
                        var cross = (p[1] - p[0]).Cross(p[2] - p[0]);
                        Assert.True(cross.Length > 0);
                        Assert.True(normal.Dot(cross) > 0);
                        Assert.InRange(normal.Length, 0.9999, 1.0001);
                        volume += p[0].Dot(p[1].Cross(p[2])) / 6;
                    }
                    double expectedVolume = 4 * Math.PI * 0.001 / 3;
                    Assert.InRange(Math.Abs(volume) / expectedVolume, 0.95, 1.05);
                }
            }
            finally { File.Delete(path); }
        }

        [Theory]
        [InlineData(1)]
        [InlineData(199999)]
        public void AtOrBelowLimitIsByteForByteUnchanged(int count)
        {
            string path = Path.GetTempFileName();
            try
            {
                using (var writer = new BinaryWriter(File.Create(path)))
                {
                    writer.Write(new byte[80]); writer.Write((uint)count);
                    for (int i = 0; i < count; i++) writer.Write(new byte[50]);
                }
                byte[] original = File.ReadAllBytes(path);
                Assert.Equal(count, StlSimplifier.LimitFaces(path, 199999));
                Assert.Equal(original, File.ReadAllBytes(path));
            }
            finally { File.Delete(path); }
        }

        [Theory]
        [InlineData(0)]
        [InlineData(3)]
        [InlineData(200000)]
        public void InvalidLimitsAreRejected(int limit)
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => StlSimplifier.LimitFaces("unused.stl", limit));
        }

        [Fact]
        public void InvalidInputIsNotOverwritten()
        {
            string path = Path.GetTempFileName();
            try
            {
                File.WriteAllText(path, "solid ascii or truncated STL");
                var original = File.ReadAllBytes(path);
                Assert.Throws<InvalidDataException>(() => StlSimplifier.LimitFaces(path, 100));
                Assert.Equal(original, File.ReadAllBytes(path));
            }
            finally { File.Delete(path); }
        }

        [Fact]
        public void UnreachableLimitKeepsOriginalMesh()
        {
            var mesh = new DMesh3();
            for (int i = 0; i < 2; i++)
            {
                int a = mesh.AppendVertex(new Vector3d(i * 2, 0, 0));
                int b = mesh.AppendVertex(new Vector3d(i * 2 + 1, 0, 0));
                int c = mesh.AppendVertex(new Vector3d(i * 2, 1, 0));
                int d = mesh.AppendVertex(new Vector3d(i * 2, 0, 1));
                mesh.AppendTriangle(a, c, b); mesh.AppendTriangle(a, b, d);
                mesh.AppendTriangle(a, d, c); mesh.AppendTriangle(b, c, d);
            }
            string path = Path.GetTempFileName();
            try
            {
                Write(path, mesh);
                byte[] original = File.ReadAllBytes(path);
                Assert.Throws<InvalidOperationException>(() => StlSimplifier.LimitFaces(path, 4));
                Assert.Equal(original, File.ReadAllBytes(path));
            }
            finally { File.Delete(path); }
        }

        [Fact]
        public void LimitSelectionIsPerLinkAndOnlyEnabledForStl()
        {
            var app = new Mock<SldWorks>(); var attribute = new Mock<AttributeDef>();
            app.Setup(x => x.DefineAttribute(It.IsAny<string>())).Returns(attribute.Object);
            using (var form = new AssemblyExportForm(app.Object, new LinkNode(), null))
            {
                var limit = (RadioButton)form.Controls.Find("radioButtonFaceLimit", true)[0];
                var count = (NumericUpDown)form.Controls.Find("numericMaximumFaces", true)[0];
                var coarse = (RadioButton)form.Controls.Find("radioButtonCourse", true)[0];
                var fine = (RadioButton)form.Controls.Find("radioButtonFine", true)[0];
                var stl = (RadioButton)form.Controls.Find("radioButtonStl", true)[0];
                var xml = (RadioButton)form.Controls.Find("radioButton3dxml", true)[0];
                var dae = (RadioButton)form.Controls.Find("radioButtonDae", true)[0];
                Assert.Equal(199999m, count.Value);
                Assert.True(coarse.Checked); Assert.False(count.Enabled);
                var first = new Link(); form.FillLinkPropertyBoxes(first);
                limit.Checked = true; count.Value = 12345;
                Assert.False(coarse.Checked); Assert.False(fine.Checked); Assert.True(count.Enabled);
                form.SaveLinkDataFromPropertyBoxes(first);
                Assert.True(first.STLUseFaceLimit); Assert.True(first.STLQualityFine); Assert.Equal(12345, first.STLMaximumFaces);
                form.FillLinkPropertyBoxes(new Link { STLMaximumFaces = 0 }); // Old saved configurations.
                Assert.True(coarse.Checked); Assert.Equal(199999m, count.Value);
                form.FillLinkPropertyBoxes(first);
                Assert.True(limit.Checked); Assert.Equal(12345m, count.Value);
                xml.Checked = true; Assert.False(limit.Enabled); Assert.False(count.Enabled);
                dae.Checked = true; Assert.False(limit.Enabled); Assert.False(count.Enabled);
                stl.Checked = true; Assert.True(limit.Enabled); Assert.True(count.Enabled);
            }
        }

        [Fact]
        public void SavedLinkRetainsLimitSettings()
        {
            var link = new Link { STLUseFaceLimit = true, STLMaximumFaces = 12345 };
            var serializer = new DataContractSerializer(typeof(Link));
            using (var stream = new MemoryStream())
            {
                serializer.WriteObject(stream, link); stream.Position = 0;
                var restored = (Link)serializer.ReadObject(stream);
                Assert.True(restored.STLUseFaceLimit); Assert.Equal(12345, restored.STLMaximumFaces);
            }
        }
    }
}
