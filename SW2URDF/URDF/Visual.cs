using System.Runtime.Serialization;

namespace SW2URDF.URDF
{
    //The visual element of a link
    [DataContract(IsReference = true, Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF")]
    public class Visual : URDFElement
    {
        [DataMember]
        public readonly Origin Origin;

        [DataMember]
        public readonly Geometry Geometry;

        [DataMember]
        public readonly Material Material;

        public Visual() : base("visual", false)
        {
            Origin = new Origin(false);
            Geometry = new Geometry();
            Material = new Material();

            ChildElements.Add(Origin);
            ChildElements.Add(Geometry);
            ChildElements.Add(Material);
        }

        public override void WriteURDF(System.Xml.XmlWriter writer)
        {
            if (!(Geometry.Mesh.Filename ?? "").EndsWith(".dae", System.StringComparison.OrdinalIgnoreCase))
            {
                base.WriteURDF(writer);
                return;
            }
            // Let consumers use the per-face colors embedded in COLLADA instead of
            // overriding every mesh material with the link's single URDF color.
            writer.WriteStartElement("visual");
            Origin.WriteURDF(writer);
            Geometry.WriteURDF(writer);
            writer.WriteEndElement();
        }
    }
}
