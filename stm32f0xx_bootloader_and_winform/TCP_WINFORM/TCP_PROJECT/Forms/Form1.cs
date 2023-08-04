using TCP_PROJECT.Common;
using TCP_PROJECT.Common.Enums;
using TCP_PROJECT.Common.Objects;
using TCP_PROJECT.Resources;
using System;
using System.Data;
using System.Data.OleDb;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Security.Cryptography;
using System.Threading;
using System.Windows.Forms;

/// <summary>
/// Windows Form App for KTC Device
/// V1.0
/// Date 11/11/2022
/// create by : Vo Quang Qua
/// Date 21/02/2023
/// update by : ThongNQ4
/// </summary>

namespace TCP_PROJECT
{

    public partial class Form1 : Form
    {
        #region Members
        DataTable table = new DataTable("table");
        int index;

        int dataInLength;
        byte[] dataIn = new byte[1000];
        byte[] targetArray = new Byte[15];

        string[] dataOut;
        string path;

        string Direcpath;
        string pathmap;

        int allowTransmission = 0, data_transfer = 0;
        byte[] hearder = new byte[256];
        byte[] imgmap;

        int transmission_array_size = 0;

        int count_value = 0;
        int clearflash = 0;
        int writedata;
        int vefifyimg = 0;
        int loadpc;

        byte[] array_data_transmission = new Byte[1000000];
        int tm = 0;
        int cycle = 0;
        int prog1Value = 0;
        int datamode = 0;
        int firmwaremode = 0;
        byte[] thu = new Byte[4];
        int sttvalue = 0;
        float tqrValue;
        byte tqrUnit;

        MySerialData serialData;

        byte direction;
        float Tartgetlow;
        float Tartgethigh;
        float maxtqr;
        byte tqrPassfall;
        int enableSave = 0;

        byte[] feedbackCmd = new byte[3];

        private static string pathSave;// = @"D:\KTC_2022\KTC_RL_SW_BLD_APP\20221109_KTC_Boothloatder\quavd.xlsx";
        // string connectionString;
        string kn = @"Provider=Microsoft.ACE.OLEDB.12.0; Data Source = " + pathSave + ";Extended Properties=\"Excel 12.0 Xml;HDR=YES;\"";


        bool g_bEndEeprom = false;
        bool g_fSendNextRecord = false;
        //bool g_bFlahResend = false;
        enum e_receiverState
        {
            idle,
            failCrc,
            receiverOk
        }
        enum e_dataMode
        {
            idle,
            readData,
            receiverData
        }

        e_receiverState receiverState = e_receiverState.idle;
        e_dataMode dataMode = e_dataMode.idle;

        #endregion

        /// <summary>
        /// Initialize Form
        /// </summary>
        #region Constructor
        public Form1()
        {
            InitializeComponent();
            timer1.Start();// Initialize time1 start
            serialData = new MySerialData();
        }
        #endregion

        #region Private methods
        

        private void UpdateLabelText()
        {
            
            lbDateUpdateFirmware.Text = DateTime.Now.ToLongDateString();// system date display for tab UpdateFirmware
            lbTimeUpdateFirmware.Text = DateTime.Now.ToLongTimeString();//system time display for tab UpdateFirmware
        }
        #endregion

        #region Events
        private void OnTimer1_Tick(object sender, EventArgs e)
        {
            UpdateLabelText();// update labeltext with time1
        }

        private void btnOpen_Click(object sender, EventArgs e)
        {
            if (btnOpen.Text == Constants.TEXT_OPEN)//  check status button btnOpen is open
            {
                try
                {
                    Array.Clear(dataIn, 0x00, dataInLength);
                    serialPort1.PortName = cBoxOpenPort.Text;
                    Properties.Settings.Default.ComPortName = serialPort1.PortName;
                    Properties.Settings.Default.Save();

                    serialPort1.Open();
                    prgUpdateFirmwareProcess.Value = prog1Value;
                    btnOpen.Text = Constants.TEXT_CLOSE;
                }
                catch (Exception err)// catch error
                {
                    MessageBox.Show(err.Message, MyResources.MessageBox_Caption_Error, MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            else //check status button btnOpen is close
            {
               
                btnOpen.Text = Constants.TEXT_OPEN;
                
                prog1Value = 0;
                prgUpdateFirmwareProcess.Value = prog1Value;
                serialPort1.Close();
                datamode = 0;
                Array.Clear(dataIn, 0x00, dataInLength);
            }
        }

        private void button4_Click(object sender, EventArgs e)
        {
            if (serialPort1.IsOpen) // check status serialPort1 is Open
            {
                try
                {
                    rtBoxUpdateFirmwareProcess.Text += "Update Firmware Start..." + Environment.NewLine;
                    ComputeStringToSha256Hash(); // perform SHA256 hashing
                    FileBitmap(); // create bitmap file
                    filesplit(); // split the data sent down to the device
                }
                catch (Exception err)
                {
                    MessageBox.Show(err.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            else // check status serialPort1 is close
            {
                MessageBox.Show("Check COM Port! ");
            }
        }

        private void button13_Click(object sender, EventArgs e)
        {
            string fileContentsave;
            string filePathsave;

            using (OpenFileDialog openFileDialog2 = new OpenFileDialog()) // initialization new OpenFileDialog
            {
                // openFileDialog2.InitialDirectory = "D:\\input_KTC\\chot_Kick_off_26_7_2022\\actionqua_ktc\\SW\\giao_dien\\KTC_SW_V1.3_30_10\\code_FW\\221004_ktc_src_boot\\src\\app\\Debug";
                // openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
                openFileDialog1.FilterIndex = 2;
                openFileDialog1.RestoreDirectory = true;

                if (openFileDialog2.ShowDialog() == DialogResult.OK)// check DialogResult
                {
                    filePathsave = openFileDialog2.FileName; //Get the filePathsave of specified file
                    
                    pathSave = filePathsave; // assign value pathSave = filePathsave
                    // connectionString = string.Format(MyResources.OLE_DB_CONNECTION_STRING, pathSave);// Connection OLE_DB with pathSave
                    kn = @"Provider=Microsoft.ACE.OLEDB.12.0; Data Source = " + pathSave + ";Extended Properties=\"Excel 12.0 Xml;HDR=YES;\"";
                }
            }
        }

       

        private void Form1_Load(object sender, EventArgs e)
        {
            string[] ports = SerialPort.GetPortNames();
            cBoxOpenPort.Items.AddRange(ports); // get serial Port 

            table.Columns.Add("NO", Type.GetType("System.String"));
            table.Columns.Add("VALUE", Type.GetType("System.String"));
            table.Columns.Add("UNIT", Type.GetType("System.String"));
            table.Columns.Add("SERIALNO", Type.GetType("System.String"));
            table.Columns.Add("DIRECTION", Type.GetType("System.String"));
            table.Columns.Add("LOW", Type.GetType("System.String"));
            table.Columns.Add("HIGH", Type.GetType("System.String"));
            table.Columns.Add("TARGET", Type.GetType("System.String"));
            table.Columns.Add("PASS/FAIL", Type.GetType("System.String"));

            cBoxOpenPort.Text  = Properties.Settings.Default.ComPortName;

        }

        private void serialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (serialPort1.IsOpen)
            {
                int i = serialPort1.BytesToRead;
                if (i == 0)
                {
                    //MessageBox.Show("i = 0");
                    return;
                }
                byte[] byte_CRC = new byte[2];
                ushort check_CRC;
                byte[] buff = new byte[i - 2];
                int k;

                Array.Clear(dataIn, 0x00, dataIn.Length);
                serialPort1.Read(dataIn, 0, i);

                for (k = 0; k < i - 2; k++)
                {
                    buff[k] = dataIn[k];
                }

                //if (dataIn[0] == 255 && dataIn[1] == 255)
                //{
                //    serialPort1.DiscardOutBuffer();
                //    serialPort1.DiscardInBuffer();
                //    return;
                //}

                check_CRC = Utils.CalcCRC16(buff, buff.Length);
                byte_CRC[0] = (byte)((check_CRC >> 8) & 0x00FF);
                byte_CRC[1] = (byte)(check_CRC & 0x00FF);

                if (buff[0] == 0x11 && byte_CRC[0] == 0xE3 && byte_CRC[1] == 0xE0)
                {
                    g_bEndEeprom = true;
                    MessageBox.Show("End Epprom");
                    return;
                }

                if (((byte_CRC[0]) == (dataIn[i - 1])) && ((byte_CRC[1]) == (dataIn[i - 2])))
                {
                    #region modeMonitoring
                    if (datamode == 1 && g_bEndEeprom == false)
                    {
                        byte[] bufffloattqrvalue = new byte[4];
                        bufffloattqrvalue[0] = buff[0];
                        bufffloattqrvalue[1] = buff[1];
                        bufffloattqrvalue[2] = buff[2];
                        bufffloattqrvalue[3] = buff[3];

                        tqrValue = System.BitConverter.ToSingle(bufffloattqrvalue, 0);

                        tqrValue = (float)Math.Round(tqrValue, 2);
                        tqrUnit = (byte)buff[4];
                        string cl2 = "";

                        switch (tqrUnit)
                        {
                            case 0: cl2 = "kgf.m"; break;
                            case 1: cl2 = "ozf.in"; break;
                            case 2: cl2 = "lbf.in"; break;
                            case 3: cl2 = "lbf.ft"; break;
                            case 4: cl2 = "cN.m"; break;
                            case 5: cl2 = "N.m"; break;
                            default: break;
                        }

                        serialData.SerialNo1 = (char)buff[5]; //1
                        serialData.SerialNo2 = (char)buff[6]; //2
                        serialData.SerialNo3 = (char)buff[7]; //3
                        serialData.SerialNo4 = (char)buff[8]; //4
                        serialData.SerialNo5 = (char)buff[9]; //5
                        serialData.SerialNo6 = (char)buff[10]; //6
                        serialData.SerialNo7 = (char)buff[11]; //7

                        //serialData.SerialNo8 = (char)buff[12];
                        //serialData.SerialNo9 = (char)buff[13];
                        //serialData.SerialNo10 = (char)buff[14];
                        //serialData.SerialNo11 = (char)buff[15];
                        //serialData.SerialNo12 = (char)buff[16];

                        serialData.SerialNo = serialData.GetSerialNo();

                        direction = (byte)buff[17];
                        string cl4 = "";

                        switch (direction)
                        {
                            case 0: cl4 = "CW"; break;
                            case 1: cl4 = "CCW"; break;
                            default: break;
                        }

                        byte[] bufffloatTargetlow = new byte[4];
                        bufffloatTargetlow[0] = buff[18];
                        bufffloatTargetlow[1] = buff[19];
                        bufffloatTargetlow[2] = buff[20];
                        bufffloatTargetlow[3] = buff[21];
                        Tartgetlow = System.BitConverter.ToSingle(bufffloatTargetlow, 0);
                        Tartgetlow = (float)Math.Round(Tartgetlow, 2);
                        byte[] bufffloatTargethigh = new byte[4];
                        bufffloatTargethigh[0] = buff[22];
                        bufffloatTargethigh[1] = buff[23];
                        bufffloatTargethigh[2] = buff[24];
                        bufffloatTargethigh[3] = buff[25];
                        Tartgethigh = System.BitConverter.ToSingle(bufffloatTargethigh, 0);
                        Tartgethigh = (float)Math.Round(Tartgethigh, 2);
                        byte[] buffmaxtqr = new byte[4];
                        buffmaxtqr[0] = buff[26];
                        buffmaxtqr[1] = buff[27];
                        buffmaxtqr[2] = buff[28];
                        buffmaxtqr[3] = buff[29];
                        maxtqr = System.BitConverter.ToSingle(buffmaxtqr, 0);
                        maxtqr = (float)Math.Round(maxtqr, 2);
                        tqrPassfall = (byte)buff[30];

                        /////////////
                        ///
                        sttvalue = sttvalue + 1;
                        string cl0 = System.Convert.ToString(sttvalue);
                        string cl1 = System.Convert.ToString(tqrValue);
                        //string cl2 = System.Convert.ToString(tqrUnit);
                        string cl3 = serialData.SerialNo;
                        //string cl4 = System.Convert.ToString(direction);
                        string cl5 = System.Convert.ToString(Tartgetlow);
                        string cl6 = System.Convert.ToString(Tartgethigh);
                        string cl7 = System.Convert.ToString(maxtqr);
                        string cl8 = "";

                        switch (tqrPassfall)
                        {
                            case 0: cl8 = "Less"; break;
                            case 1: cl8 = "Less"; break;
                            case 2: cl8 = "Less"; break;
                            case 3: cl8 = "O"; break;
                            case 4: cl8 = "O"; break;
                            case 5: cl8 = "Over"; break;
                            case 6: cl8 = "Reverse"; break;
                            default: break;
                        }

                        

                        

                        receiverState = e_receiverState.receiverOk;

                        /* Send a feedback cmd to device then received data is sent from device */
                        //                        serialPort1.Write(feedbackCmd, 0, 3);

                        if (enableSave == 1)
                        {
                            //connectionString = string.Format(MyResources.OLE_DB_CONNECTION_STRING, pathSave);
                            using (OleDbConnection conn = new OleDbConnection(kn))
                            {
                                try
                                {
                                    conn.Open();
                                    OleDbCommand cmd = new OleDbCommand();
                                    cmd.Connection = conn;
                                    //cmd.CommandText = @"Insert into[Sheet1$] (stt1,value1,unit1,serialno1,direction1,undder1,upper1,maxtqr1,passfail1) VALUES('" + cl0 + "','" + cl1 + "','" + cl2 + "','" + cl3 + "','" + cl4 + "','" + cl5 + "','" + cl6 + "','" + cl7 + "','" + cl8 + "')";
                                    cmd.CommandText = @"Insert into[Sheet1$] (NUM,VAL,UNIT,SERIALNO,DIRECTION,LOW,HIGH,TARGET,PASSFAIL) VALUES('" + cl0 + "','" + cl1 + "','" + cl2 + "','" + cl3 + "','" + cl4 + "','" + cl5 + "','" + cl6 + "','" + cl7 + "','" + cl8 + "')";
                                    cmd.ExecuteNonQuery();
                                }
                                catch (Exception ex)
                                {
                                    MessageBox.Show(ex.ToString());
                                }
                                finally
                                {
                                    conn.Close();
                                    conn.Dispose();
                                }
                            }
                        }
                    }
                    #endregion

                    #region Monitoring
                    if (buff[0] == 0x02)
                    {
                        datamode = 1;
                        //btnReceive200.Invoke(new Action(() =>
                        //{
                        //    btnReceive200.Enabled = true;
                        //}));
                    }
                    #endregion

                    #region Receiver full data form Eeprom
                    if (buff[0] == 0x03)
                    {
                        g_bEndEeprom = true;

                    }
                    #endregion

                    #region UpdateFirmware
                    if (buff[0] == 0x04 && dataMode != e_dataMode.receiverData && dataMode != e_dataMode.readData)
                    {
                        //if(dataMode != e_dataMode.receiverData)
                        datamode = 0;

                        byte[] buff_trans = new byte[6];
                        buff_trans[0] = 0x80;
                        buff_trans[1] = 0x01;
                        buff_trans[2] = 0x00;
                        buff_trans[3] = 0x10;
                        buff_trans[4] = 0xF9;
                        buff_trans[5] = 0X7C;
                        serialPort1.Write(buff_trans, 0, 6);
                        Thread.Sleep(10);
                        Array.Clear(dataIn, 0x00, dataIn.Length);
                        Array.Clear(buff_trans, 0x00, buff_trans.Length);
                        Array.Clear(buff, 0x00, buff.Length);
                        Array.Clear(byte_CRC, 0x00, byte_CRC.Length);
                        i = 0;
                        data_transfer = 1;
                        clearflash = 0;
                        writedata = 0;
                        vefifyimg = 0;
                        loadpc = 0;
                        tm = 0;
                        timer2.Stop();
                        prog1Value = 40;
                        prgUpdateFirmwareProcess.Value = prog1Value;
                    }
                    #endregion

                    #region Process UpdateFirmware
                    if (buff[0] == 0x80)
                    {
                        if (buff[3] == 0x50)
                        {
                            if (data_transfer == 1)
                            {
                                int sibyte = 0;
                                if (count_value < (transmission_array_size - 1))
                                {
                                    int sizetrbyte = (transmission_array_size - 1) - count_value;
                                    if (sizetrbyte >= 1000) { sizetrbyte = 1000; }
                                    else { sizetrbyte = sizetrbyte + 1; }
                                    byte[] array_data_buf = new Byte[sizetrbyte];
                                    for (int j = 0; j < sizetrbyte; j++)
                                    {
                                        array_data_buf[j] = array_data_transmission[j + count_value];
                                    }

                                    count_value = count_value + sizetrbyte;
                                    sibyte = sizetrbyte + 5;
                                    byte[] buff_buff_transmission = new byte[sibyte + 3];
                                    buff_buff_transmission[0] = 0x80;
                                    buff_buff_transmission[2] = (byte)((sibyte >> 8) & 0x00FF); ;
                                    buff_buff_transmission[1] = (byte)(sibyte & 0x00FF);
                                    buff_buff_transmission[3] = 0x11;

                                    int tmp = 0;

                                    if (sizetrbyte >= 1000)
                                    {
                                        tmp = 0x08007800 + cycle * sizetrbyte;
                                        cycle++;
                                    }
                                    else
                                    {
                                        tmp = 0x08007800 + (cycle) * 1000;
                                        cycle = 0;
                                    }

                                    buff_buff_transmission[4] = (Byte)((tmp >> 0) & 0x000000FF);
                                    buff_buff_transmission[5] = (Byte)((tmp >> 8) & 0x0000000FF);
                                    buff_buff_transmission[6] = (Byte)((tmp >> 16) & 0x000000FF);
                                    buff_buff_transmission[7] = (Byte)((tmp >> 24) & 0x000000FF);

                                    for (int t = 0; t < sizetrbyte; t++)
                                    {
                                        buff_buff_transmission[t + 8] = array_data_buf[t];
                                    }

                                    check_CRC = Utils.CalcCRC16(buff_buff_transmission, buff_buff_transmission.Length);

                                    byte[] buff_new_transmission = new byte[sibyte + 5];

                                    for (int h = 0; h <= (sibyte + 2); h++)
                                    {

                                        buff_new_transmission[h] = buff_buff_transmission[h];
                                    }

                                    buff_new_transmission[sibyte + 4] = (byte)((check_CRC >> 8) & 0x00FF); ;
                                    buff_new_transmission[sibyte + 3] = (byte)(check_CRC & 0x00FF);

                                    try
                                    {
                                        serialPort1.Write(buff_new_transmission, 0, buff_new_transmission.Length);
                                    }
                                    catch (Exception ex)
                                    {

                                    }

                                    Thread.Sleep(10);
                                    sizetrbyte = 0;
                                    sibyte = 0;
                                    data_transfer = 1;
                                    clearflash = 0;
                                    writedata = 0;
                                    vefifyimg = 0;
                                    loadpc = 0;
                                    prog1Value = prog1Value + 2;

                                    if (prog1Value >= 80)
                                    {
                                        prog1Value = 80;
                                    }

                                    prgUpdateFirmwareProcess.Value = prog1Value;
                                    rtBoxUpdateFirmwareProcess.Text += "Loading..." + "\n";
                                }
                                else
                                {
                                    byte[] verifyImage = new Byte[6];
                                    verifyImage[0] = 0x80;
                                    verifyImage[1] = 0x01;
                                    verifyImage[2] = 0x00;
                                    verifyImage[3] = 0x12;
                                    verifyImage[4] = 0xBB;
                                    verifyImage[5] = 0x5C;
                                    serialPort1.Write(verifyImage, 0, 6);
                                    Thread.Sleep(10);

                                    data_transfer = 2;

                                    writedata = 0;
                                    vefifyimg = 0;
                                    loadpc = 0;

                                    prog1Value = 85;

                                    prgUpdateFirmwareProcess.Value = prog1Value;
                                    rtBoxUpdateFirmwareProcess.Text += "Loading..." + "\n";
                                }
                            }
                            else
                            {
                                clearflash = 1;
                            }

                            if (clearflash == 1)
                            {
                                byte[] loadPC = new Byte[10];
                                loadPC[0] = 0x80;
                                loadPC[1] = 0x05;
                                loadPC[2] = 0x00;
                                loadPC[3] = 0x13;
                                loadPC[4] = 0x00; //address
                                loadPC[5] = 0x79; //address
                                loadPC[6] = 0x00; //address
                                loadPC[7] = 0x08; //address
                                loadPC[8] = 0x01;
                                loadPC[9] = 0x98;

                                serialPort1.Write(loadPC, 0, 10);
                                Thread.Sleep(10);
                                data_transfer = 2;
                                clearflash = 2;

                                vefifyimg = 1;
                                loadpc = 0;
                                prog1Value = 90;

                                prgUpdateFirmwareProcess.Value = prog1Value;
                                rtBoxUpdateFirmwareProcess.Text += "Loading..." + "\n";
                            }

                            if (vefifyimg == 1)
                            {
                                vefifyimg = 2;

                                prog1Value = 100;

                                prgUpdateFirmwareProcess.Value = prog1Value;
                                rtBoxUpdateFirmwareProcess.Text += "Update Firmware Successfully..." + "\n";
                            }
                        }
                        else
                        {
                            switch ((FormErrorTypes)buff[3])
                            {
                                // Magic number
                                case FormErrorTypes.Header_Error:
                                    rtBoxUpdateFirmwareProcess.Text += "Header incorrect. The packet did not begin with the required value of 0x80..." + "\n";
                                    break;

                                case FormErrorTypes.Checksum_incorrect:
                                    rtBoxUpdateFirmwareProcess.Text += "Checksum incorrect. The packet did not have the correct checksum value..." + "\n";
                                    break;

                                case FormErrorTypes.Packet_size_error:
                                    rtBoxUpdateFirmwareProcess.Text += "Packet size error..." + "\n";
                                    break;

                                case FormErrorTypes.Unknown_command:
                                    rtBoxUpdateFirmwareProcess.Text += "Unknown command..." + "\n";
                                    break;

                                case FormErrorTypes.Unknown_error:
                                    rtBoxUpdateFirmwareProcess.Text += "Unknown error..." + "\n";
                                    break;
                                case FormErrorTypes.Flash_error:
                                    rtBoxUpdateFirmwareProcess.Text += "Flash error..." + "\n";
                                    break;

                                case FormErrorTypes.Verify_image_error:
                                    rtBoxUpdateFirmwareProcess.Text += "Verify image error..." + "\n";
                                    break;

                                default:
                                    break;
                            }
                        }
                    }
                    #endregion
                }
                else
                {
                    if (dataMode != e_dataMode.receiverData)
                    {
                        MessageBox.Show("Check CRC Error! ");
                    }
                    else
                    {
                        receiverState = e_receiverState.failCrc;
                        return;

                    }
                }
            }
        }

        private void ShowData(object sender, EventArgs e)
        {

        }
      
        private void button3_Click_1(object sender, EventArgs e)
        {
            var fileContent = string.Empty;
            var filePath = string.Empty;

            using (OpenFileDialog openFileDialog1 = new OpenFileDialog())
            {
                // openFileDialog1.InitialDirectory = "D:\\input_KTC\\chot_Kick_off_26_7_2022\\actionqua_ktc\\SW\\giao_dien\\KTC_SW_V1.3_30_10\\code_FW\\221004_ktc_src_boot\\src\\app\\Debug";
                // openFileDialog1.Filter = "txt files (*.txt)|*.txt|All files (*.*)|*.*";
                openFileDialog1.FilterIndex = 2;
                openFileDialog1.RestoreDirectory = true;

                if (openFileDialog1.ShowDialog() == DialogResult.OK)
                {
                    //Get the path of specified file
                    filePath = openFileDialog1.FileName;
                }
            }

            cBoxChoseFileBin.Text = filePath;
            path = filePath;
        }

        private void ComputeStringToSha256Hash()
        {
            // Create a SHA256 hash from string   
            using (SHA256 sha256Hash = SHA256.Create())
            {
                ////////////////////////////////
                using (FileStream fileStream = new FileStream(path, FileMode.Open))
                {
                    try
                    {
                        // Create a fileStream for the file.
                        // Be sure it's positioned to the beginning of the stream.
                        fileStream.Position = 0;//set Position = 0
                        // Compute the hash of the fileStream.
                        byte[] hashValue = sha256Hash.ComputeHash(fileStream);

                        // create Header
                        int cnt = 0;
                        cnt = hashValue.Length;
                        long sizebyte = fileStream.Length;

                        byte[] version = new byte[128];
                        byte[] sizebin = new byte[4];
                        byte[] masha256 = new byte[hashValue.Length];
                        byte[] reserved = new byte[92];

                        version[0] = 0x02;
                        rtBoxUpdateFirmwareProcess.Text += "Firmware Version: 2.0" + "\n";
                        sizebin = BitConverter.GetBytes(sizebyte);

                        //sizebin = Convert.(sizebyte, 0, 4);
                        //byte.Parse(sizebyte);
                        for (int j = 0; j < 128; j++)
                        {
                            hearder[j] = version[j];
                        }

                        for (int j = 128; j < 132; j++)
                        {
                            hearder[j] = sizebin[j - 128];
                        }

                        for (int j = 132; j < 164; j++)
                        {
                            hearder[j] = hashValue[j - 132];
                        }

                        for (int j = 164; j < 256; j++)
                        {
                            //hearder[j] = reserved[j - 164];
                            hearder[j] = 0xFF;
                        }

                        prog1Value = 10;
                        prgUpdateFirmwareProcess.Value = prog1Value;
                        rtBoxUpdateFirmwareProcess.Text += "Loading..." + "\n";
                        fileStream.Position = 0;
                        fileStream.Close();
                    }
                    catch (IOException e)
                    {
                        Console.WriteLine($"I/O Exception: {e.Message}");
                    }
                    catch (UnauthorizedAccessException e)
                    {
                        Console.WriteLine($"Access Exception: {e.Message}");
                    }
                }
            }
        }

        // CreateBitmapFromHeader
        /// <summary>
        /// create File bitmap...
        /// </summary>
        private void FileBitmap()
        {

            pathmap = Path.GetDirectoryName(path) + @"\binmap";

            FileStream fs = new FileStream(pathmap, FileMode.Create);

            using (FileStream fileStream = new FileStream(path, FileMode.Open))
            {
                try
                {
                    // Create a fileStream for the file.
                    fileStream.Position = 0;
                    byte[] bytes = new byte[fileStream.Length];
                    int numBytesToRead = (int)fileStream.Length;

                    int numBytesRead = 0;

                    while (numBytesToRead > 0)
                    {
                        // Read may return anything from 0 to numBytesToRead.
                        int n = fileStream.Read(bytes, numBytesRead, numBytesToRead);

                        // Break when the end of the file is reached.
                        if (n == 0)
                        {
                            break;
                        }

                        numBytesRead += n;
                        numBytesToRead -= n;
                    }

                    numBytesToRead = bytes.Length;

                    // Write the byte array to the other FileStream.
                    fs.Write(hearder, 0, 256);
                    fs.Write(bytes, 0, numBytesToRead);
                    prog1Value = 20;
                    prgUpdateFirmwareProcess.Value = prog1Value;
                    rtBoxUpdateFirmwareProcess.Text += "Loading..." + "\n";

                    fs.Close();
                }
                catch (IOException e)
                {
                    Console.WriteLine($"I/O Exception: {e.Message}");
                }
                catch (UnauthorizedAccessException e)
                {
                    Console.WriteLine($"Access Exception: {e.Message}");
                }
            }
        }

        // CreateBitmapFromHeader
        /// <summary>
        /// split file bitmap and send data to device...
        /// </summary>
        private void filesplit()
        {
            using (FileStream fs = new FileStream(pathmap, FileMode.Open))
            {
                try
                {
                    // Create a fileStream for the file.
                    // Be sure it's positioned to the beginning of the stream.
                    fs.Position = 0;
                    byte[] bytesmap = new byte[fs.Length];
                    int numBytesToRead = (int)fs.Length;
                    int numBytesRead = 0;
                    while (numBytesToRead > 0)
                    {
                        // Read may return anything from 0 to numBytesToRead.
                        int n = fs.Read(bytesmap, numBytesRead, numBytesToRead);

                        // Break when the end of the file is reached.
                        if (n == 0)
                        {
                            break;
                        }

                        numBytesRead += n;
                        numBytesToRead -= n;
                    }

                    numBytesToRead = bytesmap.Length;
                    transmission_array_size = bytesmap.Length;
                    count_value = 0;

                    for (int i = 0; i < transmission_array_size; i++)
                    {
                        array_data_transmission[i] = bytesmap[i];
                    }

                    prog1Value = 30;
                    prgUpdateFirmwareProcess.Value = prog1Value;
                    rtBoxUpdateFirmwareProcess.Text += "Loading..." + "\n";

                    fs.Close();

                    allowTransmission = 1;

                    if ((serialPort1.IsOpen) && (allowTransmission == 1))
                    {
                        byte[] callapp = new byte[3];
                        callapp[0] = 0x05;
                        callapp[1] = 0x03;
                        callapp[2] = 0xC0;
                        serialPort1.Write(callapp, 0, 3);

                        allowTransmission = 0;
                        data_transfer = 0;
                        clearflash = 0;

                        vefifyimg = 0;
                        loadpc = 0;

                        Thread.Sleep(2000);
                        if (serialPort1.IsOpen)
                        {
                            //do nothing
                        }
                        else
                        {
                            serialPort1.Open();
                        }

                        timer2.Start();
                        tm = 0;
                    }
                }
                catch (IOException e)
                {
                    Console.WriteLine($"I/O Exception: {e.Message}");
                }
                catch (UnauthorizedAccessException e)
                {
                    Console.WriteLine($"Access Exception: {e.Message}");
                }
            }
        }

        private void timer2_Tick(object sender, EventArgs e)
        {
            tm = tm + 1;

            if (tm >= 10)
            {
                byte[] buff_transmission = new byte[6];
                buff_transmission[0] = 0x80;
                buff_transmission[1] = 0x01;
                buff_transmission[2] = 0x00;
                buff_transmission[3] = 0x10;
                buff_transmission[4] = 0xF9;
                buff_transmission[5] = 0X7C;

                serialPort1.Write(buff_transmission, 0, 6);// send command erase flash to device

                Array.Clear(buff_transmission, 0x00, buff_transmission.Length);

                data_transfer = 1;
                clearflash = 0;
                writedata = 0;
                vefifyimg = 0;
                loadpc = 0;
                tm = 0;
                timer2.Stop();
            }
        }

        

        

        private void dataGridView1_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {

        }

        private void groupBox6_Enter(object sender, EventArgs e)
        {

        }

        private void cBoxSaveFile_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void bgReceiverData_RunWorkerCompleted(object sender, System.ComponentModel.RunWorkerCompletedEventArgs e)
        {
            //bgReceiverData2.RunWorkerAsync();
        }

        private void bgReceiverData2_DoWork(object sender, System.ComponentModel.DoWorkEventArgs e)
        {
            int  coutTimer = 0;
            byte  i = 100;
            byte[] receiveCmdResend = new byte[3];
            if (serialPort1.IsOpen)// check status if serialPort1 is opening
            {
                while (i < 200)
                {
                    //if (g_bEndEeprom == true)
                    //{
                    //    break;
                    //}

                    if (receiverState == e_receiverState.receiverOk || receiverState == e_receiverState.failCrc)
                    {
                        serialPort1.DiscardOutBuffer();
                        serialPort1.DiscardInBuffer();
                        receiverState = e_receiverState.idle;
                        byte[] receiveCmd = new byte[3];// create array
                        receiveCmd[0] = 0x10; // ReceiveData cmd
                        ushort crc16ReceiCmd = Utils.CalcCRC16(receiveCmd, 1);
                        
                        receiveCmd[1] = (byte)(crc16ReceiCmd & 0xff); // CRC Low (0xC9)
                        receiveCmd[2] = (byte)(crc16ReceiCmd >> 8); // CRC High (0x72)
                        serialPort1.Write(receiveCmd, 0, 3); // send Command to device to inform that want receive 200 data from devic

                        receiveCmdResend = receiveCmd;
                        i++;
                    }

                    while (true)
                    {
                        Thread.Sleep(10);
                        if (receiverState == e_receiverState.receiverOk || receiverState == e_receiverState.failCrc)
                        {
                            break;
                        }
                        coutTimer++;
                        if (coutTimer >= 20)
                        {
                            coutTimer = 0;
                            //e_receiverState.receiverOk;
                            break;
                        }
                    }
                }
            }
        }


        private void tBoxDataIn_TextChanged(object sender, EventArgs e)
        {

        }
        #endregion
    }
}
