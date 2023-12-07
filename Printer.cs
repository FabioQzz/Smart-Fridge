using System;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Utilities;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class Printer : IPeripheral, IBytePeripheral
    {		
        public void Reset()
        {
            // nothing happens
        }

        public byte ReadByte(long offset)
        {
            byte ret = 0;
            return ret;
        }

        public void WriteByte(long offset, byte value)
        {
            Console.Write("{0}", Convert.ToChar(value));
        }
    }
}