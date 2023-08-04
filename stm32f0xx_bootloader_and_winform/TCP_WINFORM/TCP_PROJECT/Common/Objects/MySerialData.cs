
using System;

namespace TCP_PROJECT.Common.Objects
{
    public class MySerialData
    {
        public char SerialNo1 { get; set; }
        public char SerialNo2 { get; set; }
        public char SerialNo3 { get; set; }
        public char SerialNo4 { get; set; }
        public char SerialNo5 { get; set; }
        public char SerialNo6 { get; set; }
        public char SerialNo7 { get; set; }
        //public char SerialNo8 { get; set; }
        //public char SerialNo9 { get; set; }
        //public char SerialNo10 { get; set; }
        //public char SerialNo11 { get; set; }
        //public char SerialNo12 { get; set; }

        public String SerialNo { get; set; }

        public MySerialData()
        {

        }

        public string GetSerialNo()
        {
            return System.Convert.ToString(this.SerialNo1)
                            + System.Convert.ToString(this.SerialNo2)
                            + System.Convert.ToString(this.SerialNo3)
                            + System.Convert.ToString(this.SerialNo4)
                            + System.Convert.ToString(this.SerialNo5)
                            + System.Convert.ToString(this.SerialNo6)
                            + System.Convert.ToString(this.SerialNo7);
                            //+ System.Convert.ToString(this.SerialNo8)
                            //+ System.Convert.ToString(this.SerialNo9)
                            //+ System.Convert.ToString(this.SerialNo10)
                            //+ System.Convert.ToString(this.SerialNo11)
                            //+ System.Convert.ToString(this.SerialNo12);
        }

        public void SetSerialData(byte[] buff)
        {

        }
    }
}
