using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml;

namespace SW2URDF.URDFExport
{
    // A small COLLADA 1.4.1 writer: triangles, normals and opaque diffuse colors only.
    internal sealed class ColladaMesh
    {
        private const string Namespace = "http://www.collada.org/2005/11/COLLADASchema";
        private sealed class ColorGroup
        {
            public double[] Color;
            public readonly List<double> Positions = new List<double>();
            public readonly List<double> Normals = new List<double>();
        }
        private readonly List<ColorGroup> groups = new List<ColorGroup>();

        internal static double[] SelectColor(params double[][] candidates)
        {
            foreach (double[] candidate in candidates)
                if (candidate != null && candidate.Length >= 3 &&
                    candidate.Take(3).All(x => !double.IsNaN(x) && x >= 0 && x <= 1))
                    return candidate.Take(3).ToArray();
            return new double[] { 0.75, 0.75, 0.75 };
        }

        internal void AddFace(float[] triangles, float[] normals, Matrix<double> transform, double[] color)
        {
            if (triangles == null || triangles.Length == 0 || triangles.Length % 9 != 0)
                throw new InvalidDataException("No valid face tessellation. Resolve the component and rebuild the model before exporting DAE.");
            if (triangles.Any(x => float.IsNaN(x) || float.IsInfinity(x)))
                throw new InvalidDataException("The face tessellation contains invalid coordinates.");
            color = SelectColor(color);
            ColorGroup group = groups.FirstOrDefault(g => g.Color.SequenceEqual(color));
            if (group == null) { group = new ColorGroup { Color = color }; groups.Add(group); }
            Matrix<double> linear = transform.SubMatrix(0, 3, 0, 3);
            Matrix<double> normalTransform = linear.Inverse().Transpose();
            bool mirrored = linear.Determinant() < 0;
            for (int triangle = 0; triangle < triangles.Length; triangle += 9)
            {
                var points = new Vector<double>[3];
                for (int vertex = 0; vertex < 3; vertex++)
                {
                    int offset = triangle + vertex * 3;
                    points[vertex] = transform * DenseVector.OfArray(new double[] {
                        triangles[offset], triangles[offset + 1], triangles[offset + 2], 1 });
                }
                int[] order = mirrored ? new[] { 0, 2, 1 } : new[] { 0, 1, 2 };
                var u = points[order[1]] - points[0];
                var v = points[order[2]] - points[0];
                Vector<double> flatNormal = DenseVector.OfArray(new[] {
                    u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0] });
                if (flatNormal.L2Norm() < 1e-20) continue;
                flatNormal = flatNormal.Normalize(2);
                foreach (int vertex in order)
                {
                    int offset = triangle + vertex * 3;
                    Vector<double> normal = flatNormal;
                    if (normals != null && normals.Length == triangles.Length)
                    {
                        var transformed = normalTransform * DenseVector.OfArray(new double[] {
                            normals[offset], normals[offset + 1], normals[offset + 2] });
                        if (transformed.All(x => !double.IsNaN(x) && !double.IsInfinity(x)) && transformed.L2Norm() > 1e-20)
                            normal = transformed.Normalize(2);
                    }
                    group.Positions.AddRange(points[vertex].Take(3));
                    group.Normals.AddRange(normal);
                }
            }
        }

        internal void Write(string path)
        {
            var nonempty = groups.Where(g => g.Positions.Count > 0).ToList();
            if (nonempty.Count == 0) throw new InvalidDataException("The link has no triangles to export as DAE.");
            using (var writer = XmlWriter.Create(path, new XmlWriterSettings { Encoding = new UTF8Encoding(false), Indent = true }))
            {
                writer.WriteStartElement("COLLADA", Namespace);
                writer.WriteAttributeString("version", "1.4.1");
                Start(writer, "asset");
                string timestamp = XmlConvert.ToString(DateTime.UtcNow, XmlDateTimeSerializationMode.Utc);
                Element(writer, "created", timestamp); Element(writer, "modified", timestamp);
                Start(writer, "unit", "name", "meter", "meter", "1"); writer.WriteEndElement();
                Element(writer, "up_axis", "Z_UP"); writer.WriteEndElement();
                Start(writer, "library_effects");
                for (int i = 0; i < nonempty.Count; i++)
                {
                    Start(writer, "effect", "id", "effect" + i);
                    Start(writer, "profile_COMMON"); Start(writer, "technique", "sid", "common");
                    Start(writer, "lambert"); Start(writer, "diffuse");
                    Element(writer, "color", string.Join(" ", nonempty[i].Color.Select(Number)) + " 1");
                    for (int j = 0; j < 5; j++) writer.WriteEndElement();
                }
                writer.WriteEndElement();
                Start(writer, "library_materials");
                for (int i = 0; i < nonempty.Count; i++)
                {
                    Start(writer, "material", "id", "material" + i);
                    Start(writer, "instance_effect", "url", "#effect" + i); writer.WriteEndElement(); writer.WriteEndElement();
                }
                writer.WriteEndElement();
                Start(writer, "library_geometries");
                for (int i = 0; i < nonempty.Count; i++)
                {
                    string id = "geometry" + i;
                    var group = nonempty[i];
                    Start(writer, "geometry", "id", id); Start(writer, "mesh");
                    WriteSource(writer, id + "-positions", group.Positions);
                    WriteSource(writer, id + "-normals", group.Normals);
                    Start(writer, "vertices", "id", id + "-vertices");
                    Start(writer, "input", "semantic", "POSITION", "source", "#" + id + "-positions");
                    writer.WriteEndElement(); writer.WriteEndElement();
                    Start(writer, "triangles", "count", (group.Positions.Count / 9).ToString(CultureInfo.InvariantCulture), "material", "color" + i);
                    Start(writer, "input", "semantic", "VERTEX", "source", "#" + id + "-vertices", "offset", "0"); writer.WriteEndElement();
                    Start(writer, "input", "semantic", "NORMAL", "source", "#" + id + "-normals", "offset", "0"); writer.WriteEndElement();
                    Start(writer, "p");
                    for (int j = 0; j < group.Positions.Count / 3; j++) writer.WriteString(j.ToString(CultureInfo.InvariantCulture) + " ");
                    writer.WriteEndElement(); writer.WriteEndElement(); writer.WriteEndElement(); writer.WriteEndElement();
                }
                writer.WriteEndElement();
                Start(writer, "library_visual_scenes"); Start(writer, "visual_scene", "id", "Scene");
                for (int i = 0; i < nonempty.Count; i++)
                {
                    Start(writer, "node", "id", "node" + i);
                    Start(writer, "instance_geometry", "url", "#geometry" + i);
                    Start(writer, "bind_material"); Start(writer, "technique_common");
                    Start(writer, "instance_material", "symbol", "color" + i, "target", "#material" + i);
                    for (int j = 0; j < 5; j++) writer.WriteEndElement();
                }
                writer.WriteEndElement(); writer.WriteEndElement();
                Start(writer, "scene"); Start(writer, "instance_visual_scene", "url", "#Scene");
                writer.WriteEndElement(); writer.WriteEndElement(); writer.WriteEndElement();
            }
        }

        private static string Number(double value) => value.ToString("R", CultureInfo.InvariantCulture);
        private static void Start(XmlWriter writer, string name, params string[] attributes)
        {
            writer.WriteStartElement(name, Namespace);
            for (int i = 0; i < attributes.Length; i += 2) writer.WriteAttributeString(attributes[i], attributes[i + 1]);
        }
        private static void Element(XmlWriter writer, string name, string value)
        {
            Start(writer, name); writer.WriteString(value); writer.WriteEndElement();
        }
        private static void WriteSource(XmlWriter writer, string id, List<double> values)
        {
            Start(writer, "source", "id", id);
            Start(writer, "float_array", "id", id + "-array", "count", values.Count.ToString(CultureInfo.InvariantCulture));
            foreach (double value in values) writer.WriteString(Number(value) + " ");
            writer.WriteEndElement(); Start(writer, "technique_common");
            Start(writer, "accessor", "source", "#" + id + "-array", "count", (values.Count / 3).ToString(CultureInfo.InvariantCulture), "stride", "3");
            foreach (string axis in new[] { "X", "Y", "Z" })
            { Start(writer, "param", "name", axis, "type", "float"); writer.WriteEndElement(); }
            writer.WriteEndElement(); writer.WriteEndElement(); writer.WriteEndElement();
        }
    }
}
