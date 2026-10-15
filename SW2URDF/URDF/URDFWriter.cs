using System.Text;
using System.Xml;
using System;
using System.IO;

namespace SW2URDF.URDF
{
    //Initiates the XMLWriter and its necessary settings
    public class URDFWriter
    {
        public XmlWriter writer;

        public URDFWriter(string savePath)
        {
            XmlWriterSettings settings = new XmlWriterSettings
            {
                Encoding = new UTF8Encoding(false),
                Indent = true,
                NewLineOnAttributes = true,
            };
            writer = XmlWriter.Create(savePath, settings);
        }

        // Reformat generated robot XML after its writer has closed. Parse XML rather
        // than replacing whitespace in strings, so attribute values and text survive.
        public static void FormatFile(string path)
        {
            var document = new XmlDocument { XmlResolver = null };
            var readerSettings = new XmlReaderSettings
            {
                DtdProcessing = DtdProcessing.Prohibit,
                XmlResolver = null,
                IgnoreWhitespace = true
            };
            using (var reader = XmlReader.Create(path, readerSettings))
            {
                document.Load(reader);
            }

            var settings = new XmlWriterSettings
            {
                Encoding = new UTF8Encoding(false),
                Indent = true,
                IndentChars = "  ",
                NewLineChars = "\n",
                NewLineOnAttributes = false,
                NewLineHandling = NewLineHandling.Entitize,
                OmitXmlDeclaration = !(document.FirstChild is XmlDeclaration)
            };
            // Keep the original intact if parsing or writing fails. A sibling temp
            // file permits replacement on the same filesystem after a complete write.
            string temporaryPath = path + "." + Guid.NewGuid().ToString("N") + ".tmp";
            try
            {
                using (var formattedWriter = XmlWriter.Create(temporaryPath, settings))
                {
                    document.Save(formattedWriter);
                }
                File.Replace(temporaryPath, path, null);
            }
            finally
            {
                if (File.Exists(temporaryPath)) File.Delete(temporaryPath);
            }
        }
    }
}
