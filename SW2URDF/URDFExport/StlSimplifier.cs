using g3;
using System;
using System.Collections.Generic;
using System.IO;

namespace SW2URDF.URDFExport
{
    internal static class StlSimplifier
    {
        internal const int DefaultMaximumFaces = 199999;
        private static readonly log4net.ILog logger = Utilities.Logger.GetLogger();

        internal static int LimitFaces(string filename, int maximumFaces)
        {
            if (maximumFaces < 4 || maximumFaces > DefaultMaximumFaces)
                throw new ArgumentOutOfRangeException(nameof(maximumFaces));
            int originalCount = ReadTriangleCount(filename);
            if (originalCount <= maximumFaces) return originalCount;
            logger.Info("Simplifying STL " + filename + " from " + originalCount + " faces to at most " + maximumFaces + " using QEM");
            DMesh3 mesh = ReadMesh(filename, originalCount);
            if (mesh.TriangleCount == 0) throw new InvalidDataException("The STL contains no non-degenerate triangles.");
            // STL has no connectivity. Coincident parts can weld to bowtie vertices;
            // split those vertices and orient adjacent faces without removing geometry.
            new gs.MeshRepairOrientation(mesh).OrientComponents();
            new MeshEditor(mesh).DisconnectAllBowties(10);
            if (!mesh.CheckValidity(false, FailMode.ReturnOnly))
                throw new InvalidDataException("Cannot reconstruct valid STL connectivity for simplification.");
            var reducer = new Reducer(mesh) { PreserveBoundaryShape = true };
            reducer.ReduceToTriangleCount(maximumFaces);
            if (mesh.TriangleCount == 0 || mesh.TriangleCount > maximumFaces)
                throw new InvalidOperationException("Cannot simplify STL to the requested face limit while preserving valid mesh connectivity. Remaining faces: " + mesh.TriangleCount);
            if (!mesh.CheckValidity(true, FailMode.ReturnOnly))
                throw new InvalidDataException("STL simplification produced invalid mesh connectivity.");
            string temporary = filename + "." + Guid.NewGuid().ToString("N") + ".tmp";
            int outputCount;
            try
            {
                WriteMesh(temporary, mesh);
                outputCount = ReadTriangleCount(temporary);
                if (outputCount > maximumFaces) throw new InvalidDataException("STL face limit exceeded.");
                File.Replace(temporary, filename, null);
            }
            finally { if (File.Exists(temporary)) File.Delete(temporary); }
            logger.Info("Simplified STL " + filename + ": " + originalCount + " -> " + outputCount + " faces");
            return outputCount;
        }

        internal static int ReadTriangleCount(string filename)
        {
            using (var reader = new BinaryReader(File.OpenRead(filename)))
            {
                if (reader.BaseStream.Length < 84) throw new InvalidDataException("Truncated binary STL: " + filename);
                reader.BaseStream.Position = 80;
                uint count = reader.ReadUInt32();
                if (count == 0 || count > int.MaxValue || reader.BaseStream.Length != 84L + 50L * count)
                    throw new InvalidDataException("Invalid binary STL face count or file length: " + filename);
                return (int)count;
            }
        }

        private static DMesh3 ReadMesh(string filename, int count)
        {
            var builder = new DMesh3Builder
            {
                DuplicateTriBehavior = DMesh3Builder.AddTriangleFailBehaviors.DuplicateAllVertices,
                NonManifoldTriBehavior = DMesh3Builder.AddTriangleFailBehaviors.DuplicateAllVertices
            };
            builder.AppendNewMesh(false, false, false, false);
            var vertices = new Dictionary<Vector3d, int>();
            using (var reader = new BinaryReader(File.OpenRead(filename)))
            {
                reader.BaseStream.Position = 84;
                for (int triangle = 0; triangle < count; triangle++)
                {
                    reader.BaseStream.Position += 12;
                    Vector3d a = ReadVertex(reader), b = ReadVertex(reader), c = ReadVertex(reader);
                    reader.ReadUInt16();
                    if (a == b || b == c || c == a || (b - a).Cross(c - a).LengthSquared == 0) continue;
                    if (builder.AppendTriangle(GetVertex(builder, vertices, a), GetVertex(builder, vertices, b), GetVertex(builder, vertices, c)) < 0)
                        throw new InvalidDataException("Cannot reconstruct STL triangle connectivity.");
                }
            }
            return builder.Meshes[0];
        }

        private static Vector3d ReadVertex(BinaryReader reader)
        {
            var vertex = new Vector3d(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
            Validate(vertex);
            return vertex;
        }
        private static void Validate(Vector3d vertex)
        {
            if (double.IsNaN(vertex.x) || double.IsInfinity(vertex.x) || double.IsNaN(vertex.y) || double.IsInfinity(vertex.y) || double.IsNaN(vertex.z) || double.IsInfinity(vertex.z))
                throw new InvalidDataException("STL contains a non-finite vertex coordinate.");
        }
        private static int GetVertex(DMesh3Builder builder, Dictionary<Vector3d, int> vertices, Vector3d vertex)
        {
            if (!vertices.TryGetValue(vertex, out int id))
            {
                id = builder.AppendVertex(vertex.x, vertex.y, vertex.z);
                vertices.Add(vertex, id);
            }
            return id;
        }

        private static void WriteMesh(string filename, DMesh3 mesh)
        {
            using (var writer = new BinaryWriter(File.Create(filename)))
            {
                writer.Write(new byte[80]); writer.Write((uint)mesh.TriangleCount);
                uint written = 0;
                foreach (int id in mesh.TriangleIndices())
                {
                    Index3i triangle = mesh.GetTriangle(id);
                    Vector3d a = ToSingle(mesh.GetVertex(triangle.a));
                    Vector3d b = ToSingle(mesh.GetVertex(triangle.b));
                    Vector3d c = ToSingle(mesh.GetVertex(triangle.c));
                    Vector3d normal = (b - a).Cross(c - a);
                    // Float conversion can collapse a very small triangle. Such zero-area
                    // faces have no surface and must not be passed to a physics importer.
                    if (normal.Normalize(0) == 0) continue;
                    WriteVertex(writer, normal); WriteVertex(writer, a); WriteVertex(writer, b); WriteVertex(writer, c);
                    writer.Write((ushort)0);
                    written++;
                }
                writer.BaseStream.Position = 80;
                writer.Write(written);
                if (written != mesh.TriangleCount)
                    logger.Info("Removed " + (mesh.TriangleCount - written) + " zero-area faces after STL float conversion");
            }
        }
        private static Vector3d ToSingle(Vector3d vertex)
        {
            var result = new Vector3d((float)vertex.x, (float)vertex.y, (float)vertex.z);
            Validate(result);
            return result;
        }
        private static void WriteVertex(BinaryWriter writer, Vector3d vertex)
        {
            writer.Write((float)vertex.x); writer.Write((float)vertex.y); writer.Write((float)vertex.z);
        }
    }
}
