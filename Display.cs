using System;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Utilities;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class Display : IPeripheral, IDoubleWordPeripheral
    {		
        public void Reset()
        {
            // nothing happens
        }

        public uint ReadDoubleWord(long offset)
        {
            
            uint ret = 0;
            return ret;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            Console.Write("{0}", Convert.ToChar(value));
        }
    }
}