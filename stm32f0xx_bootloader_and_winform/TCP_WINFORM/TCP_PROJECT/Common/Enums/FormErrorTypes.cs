using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TCP_PROJECT.Common.Enums
{
    /// <summary>
    /// Define NACK
    /// </summary>
    internal enum FormErrorTypes
    {
        Header_Error = 0x51,
        Checksum_incorrect =0x52,
        Packet_size_error = 0x53,
        Unknown_command = 0x54,
        Unknown_error = 0x55,
        Flash_error = 0x56,
        Verify_image_error = 0x57

    }
}
