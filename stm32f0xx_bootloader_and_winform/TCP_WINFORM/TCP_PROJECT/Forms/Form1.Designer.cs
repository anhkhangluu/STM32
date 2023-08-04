namespace TCP_PROJECT
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.openFileDialog1 = new System.Windows.Forms.OpenFileDialog();
            this.tabPage3 = new System.Windows.Forms.TabPage();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.lbTimeUpdateFirmware = new System.Windows.Forms.Label();
            this.tableLayoutPanel3 = new System.Windows.Forms.TableLayoutPanel();
            this.label67 = new System.Windows.Forms.Label();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.cBoxChoseFileBin = new System.Windows.Forms.ComboBox();
            this.label69 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.btnChoseFileBin = new System.Windows.Forms.Button();
            this.prgUpdateFirmwareProcess = new System.Windows.Forms.ProgressBar();
            this.btnUpdateFirmwareProcess = new System.Windows.Forms.Button();
            this.label70 = new System.Windows.Forms.Label();
            this.tableLayoutPanel2 = new System.Windows.Forms.TableLayoutPanel();
            this.btnOpen = new System.Windows.Forms.Button();
            this.cBoxOpenPort = new System.Windows.Forms.ComboBox();
            this.lbDateUpdateFirmware = new System.Windows.Forms.Label();
            this.tableLayoutPanel1 = new System.Windows.Forms.TableLayoutPanel();
            this.rtBoxUpdateFirmwareProcess = new System.Windows.Forms.RichTextBox();
            this.label73 = new System.Windows.Forms.Label();
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.timer2 = new System.Windows.Forms.Timer(this.components);
            this.openFileDialog2 = new System.Windows.Forms.OpenFileDialog();
            this.bgReceiverData = new System.ComponentModel.BackgroundWorker();
            this.bgReceiverData2 = new System.ComponentModel.BackgroundWorker();
            this.tabPage3.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.tableLayoutPanel3.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.tableLayoutPanel2.SuspendLayout();
            this.tableLayoutPanel1.SuspendLayout();
            this.tabControl1.SuspendLayout();
            this.SuspendLayout();
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.OnTimer1_Tick);
            // 
            // serialPort1
            // 
            this.serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serialPort1_DataReceived);
            // 
            // openFileDialog1
            // 
            this.openFileDialog1.FileName = "openFileDialog1";
            // 
            // tabPage3
            // 
            this.tabPage3.Controls.Add(this.groupBox6);
            this.tabPage3.Location = new System.Drawing.Point(4, 22);
            this.tabPage3.Name = "tabPage3";
            this.tabPage3.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage3.Size = new System.Drawing.Size(897, 518);
            this.tabPage3.TabIndex = 2;
            this.tabPage3.Text = "UPDATE FIRMWARE";
            this.tabPage3.UseVisualStyleBackColor = true;
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.lbTimeUpdateFirmware);
            this.groupBox6.Controls.Add(this.tableLayoutPanel3);
            this.groupBox6.Controls.Add(this.lbDateUpdateFirmware);
            this.groupBox6.Controls.Add(this.tableLayoutPanel1);
            this.groupBox6.Dock = System.Windows.Forms.DockStyle.Fill;
            this.groupBox6.Location = new System.Drawing.Point(3, 3);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Size = new System.Drawing.Size(891, 512);
            this.groupBox6.TabIndex = 0;
            this.groupBox6.TabStop = false;
            this.groupBox6.Enter += new System.EventHandler(this.groupBox6_Enter);
            // 
            // lbTimeUpdateFirmware
            // 
            this.lbTimeUpdateFirmware.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.lbTimeUpdateFirmware.AutoSize = true;
            this.lbTimeUpdateFirmware.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbTimeUpdateFirmware.Location = new System.Drawing.Point(792, 485);
            this.lbTimeUpdateFirmware.Name = "lbTimeUpdateFirmware";
            this.lbTimeUpdateFirmware.Size = new System.Drawing.Size(33, 16);
            this.lbTimeUpdateFirmware.TabIndex = 35;
            this.lbTimeUpdateFirmware.Text = "time";
            // 
            // tableLayoutPanel3
            // 
            this.tableLayoutPanel3.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanel3.ColumnCount = 2;
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 31.38253F));
            this.tableLayoutPanel3.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 68.61747F));
            this.tableLayoutPanel3.Controls.Add(this.label67, 0, 0);
            this.tableLayoutPanel3.Controls.Add(this.groupBox4, 1, 1);
            this.tableLayoutPanel3.Controls.Add(this.label70, 1, 0);
            this.tableLayoutPanel3.Controls.Add(this.tableLayoutPanel2, 0, 1);
            this.tableLayoutPanel3.Location = new System.Drawing.Point(3, 10);
            this.tableLayoutPanel3.Margin = new System.Windows.Forms.Padding(2);
            this.tableLayoutPanel3.Name = "tableLayoutPanel3";
            this.tableLayoutPanel3.RowCount = 2;
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 13.56784F));
            this.tableLayoutPanel3.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 86.43216F));
            this.tableLayoutPanel3.Size = new System.Drawing.Size(882, 116);
            this.tableLayoutPanel3.TabIndex = 44;
            // 
            // label67
            // 
            this.label67.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right)));
            this.label67.AutoSize = true;
            this.label67.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label67.ForeColor = System.Drawing.SystemColors.Highlight;
            this.label67.Location = new System.Drawing.Point(3, 1);
            this.label67.Name = "label67";
            this.label67.Size = new System.Drawing.Size(270, 13);
            this.label67.TabIndex = 38;
            this.label67.Text = "COM PORT CONTROL";
            this.label67.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // groupBox4
            // 
            this.groupBox4.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.groupBox4.Controls.Add(this.cBoxChoseFileBin);
            this.groupBox4.Controls.Add(this.label69);
            this.groupBox4.Controls.Add(this.label13);
            this.groupBox4.Controls.Add(this.btnChoseFileBin);
            this.groupBox4.Controls.Add(this.prgUpdateFirmwareProcess);
            this.groupBox4.Controls.Add(this.btnUpdateFirmwareProcess);
            this.groupBox4.Location = new System.Drawing.Point(278, 17);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(2);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(2);
            this.groupBox4.Size = new System.Drawing.Size(602, 97);
            this.groupBox4.TabIndex = 43;
            this.groupBox4.TabStop = false;
            // 
            // cBoxChoseFileBin
            // 
            this.cBoxChoseFileBin.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.cBoxChoseFileBin.FormattingEnabled = true;
            this.cBoxChoseFileBin.Location = new System.Drawing.Point(122, 20);
            this.cBoxChoseFileBin.Name = "cBoxChoseFileBin";
            this.cBoxChoseFileBin.Size = new System.Drawing.Size(354, 21);
            this.cBoxChoseFileBin.TabIndex = 41;
            // 
            // label69
            // 
            this.label69.AutoSize = true;
            this.label69.Location = new System.Drawing.Point(16, 74);
            this.label69.Name = "label69";
            this.label69.Size = new System.Drawing.Size(51, 13);
            this.label69.TabIndex = 42;
            this.label69.Text = "Process :";
            // 
            // label13
            // 
            this.label13.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(16, 24);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(89, 13);
            this.label13.TabIndex = 34;
            this.label13.Text = "Curent Directory :";
            // 
            // btnChoseFileBin
            // 
            this.btnChoseFileBin.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.btnChoseFileBin.Location = new System.Drawing.Point(482, 18);
            this.btnChoseFileBin.Name = "btnChoseFileBin";
            this.btnChoseFileBin.Size = new System.Drawing.Size(97, 23);
            this.btnChoseFileBin.TabIndex = 35;
            this.btnChoseFileBin.Text = "Browse";
            this.btnChoseFileBin.UseVisualStyleBackColor = true;
            this.btnChoseFileBin.Click += new System.EventHandler(this.button3_Click_1);
            // 
            // prgUpdateFirmwareProcess
            // 
            this.prgUpdateFirmwareProcess.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.prgUpdateFirmwareProcess.Location = new System.Drawing.Point(122, 67);
            this.prgUpdateFirmwareProcess.Name = "prgUpdateFirmwareProcess";
            this.prgUpdateFirmwareProcess.Size = new System.Drawing.Size(353, 22);
            this.prgUpdateFirmwareProcess.Step = 1;
            this.prgUpdateFirmwareProcess.TabIndex = 33;
            // 
            // btnUpdateFirmwareProcess
            // 
            this.btnUpdateFirmwareProcess.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.btnUpdateFirmwareProcess.Location = new System.Drawing.Point(482, 67);
            this.btnUpdateFirmwareProcess.Name = "btnUpdateFirmwareProcess";
            this.btnUpdateFirmwareProcess.Size = new System.Drawing.Size(97, 23);
            this.btnUpdateFirmwareProcess.TabIndex = 36;
            this.btnUpdateFirmwareProcess.Text = "Update Firmware";
            this.btnUpdateFirmwareProcess.UseVisualStyleBackColor = true;
            this.btnUpdateFirmwareProcess.Click += new System.EventHandler(this.button4_Click);
            // 
            // label70
            // 
            this.label70.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right)));
            this.label70.AutoSize = true;
            this.label70.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label70.ForeColor = System.Drawing.SystemColors.Highlight;
            this.label70.Location = new System.Drawing.Point(279, 1);
            this.label70.Name = "label70";
            this.label70.Size = new System.Drawing.Size(600, 13);
            this.label70.TabIndex = 40;
            this.label70.Text = "UPDATE FIRMWARE";
            this.label70.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tableLayoutPanel2
            // 
            this.tableLayoutPanel2.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanel2.ColumnCount = 1;
            this.tableLayoutPanel2.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Percent, 100F));
            this.tableLayoutPanel2.Controls.Add(this.btnOpen, 0, 1);
            this.tableLayoutPanel2.Controls.Add(this.cBoxOpenPort, 0, 0);
            this.tableLayoutPanel2.Location = new System.Drawing.Point(2, 17);
            this.tableLayoutPanel2.Margin = new System.Windows.Forms.Padding(2);
            this.tableLayoutPanel2.Name = "tableLayoutPanel2";
            this.tableLayoutPanel2.RowCount = 2;
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel2.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 50F));
            this.tableLayoutPanel2.Size = new System.Drawing.Size(272, 97);
            this.tableLayoutPanel2.TabIndex = 44;
            // 
            // btnOpen
            // 
            this.btnOpen.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.btnOpen.BackColor = System.Drawing.Color.Gainsboro;
            this.btnOpen.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnOpen.ForeColor = System.Drawing.Color.Black;
            this.btnOpen.Location = new System.Drawing.Point(3, 51);
            this.btnOpen.Name = "btnOpen";
            this.btnOpen.Size = new System.Drawing.Size(266, 43);
            this.btnOpen.TabIndex = 8;
            this.btnOpen.Text = "OPEN";
            this.btnOpen.UseVisualStyleBackColor = false;
            this.btnOpen.Click += new System.EventHandler(this.btnOpen_Click);
            // 
            // cBoxOpenPort
            // 
            this.cBoxOpenPort.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.cBoxOpenPort.Font = new System.Drawing.Font("Microsoft Sans Serif", 13F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.cBoxOpenPort.FormattingEnabled = true;
            this.cBoxOpenPort.Location = new System.Drawing.Point(3, 3);
            this.cBoxOpenPort.Name = "cBoxOpenPort";
            this.cBoxOpenPort.Size = new System.Drawing.Size(266, 28);
            this.cBoxOpenPort.TabIndex = 7;
            // 
            // lbDateUpdateFirmware
            // 
            this.lbDateUpdateFirmware.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left)));
            this.lbDateUpdateFirmware.AutoSize = true;
            this.lbDateUpdateFirmware.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.lbDateUpdateFirmware.Location = new System.Drawing.Point(3, 485);
            this.lbDateUpdateFirmware.Name = "lbDateUpdateFirmware";
            this.lbDateUpdateFirmware.Size = new System.Drawing.Size(35, 16);
            this.lbDateUpdateFirmware.TabIndex = 34;
            this.lbDateUpdateFirmware.Text = "date";
            // 
            // tableLayoutPanel1
            // 
            this.tableLayoutPanel1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tableLayoutPanel1.ColumnCount = 1;
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle());
            this.tableLayoutPanel1.ColumnStyles.Add(new System.Windows.Forms.ColumnStyle(System.Windows.Forms.SizeType.Absolute, 20F));
            this.tableLayoutPanel1.Controls.Add(this.rtBoxUpdateFirmwareProcess, 0, 0);
            this.tableLayoutPanel1.Controls.Add(this.label73, 0, 1);
            this.tableLayoutPanel1.Location = new System.Drawing.Point(2, 143);
            this.tableLayoutPanel1.Margin = new System.Windows.Forms.Padding(2);
            this.tableLayoutPanel1.Name = "tableLayoutPanel1";
            this.tableLayoutPanel1.RowCount = 2;
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 94.28571F));
            this.tableLayoutPanel1.RowStyles.Add(new System.Windows.Forms.RowStyle(System.Windows.Forms.SizeType.Percent, 5.714286F));
            this.tableLayoutPanel1.Size = new System.Drawing.Size(883, 340);
            this.tableLayoutPanel1.TabIndex = 2;
            // 
            // rtBoxUpdateFirmwareProcess
            // 
            this.rtBoxUpdateFirmwareProcess.Dock = System.Windows.Forms.DockStyle.Fill;
            this.rtBoxUpdateFirmwareProcess.Location = new System.Drawing.Point(3, 3);
            this.rtBoxUpdateFirmwareProcess.Name = "rtBoxUpdateFirmwareProcess";
            this.rtBoxUpdateFirmwareProcess.Size = new System.Drawing.Size(877, 314);
            this.rtBoxUpdateFirmwareProcess.TabIndex = 1;
            this.rtBoxUpdateFirmwareProcess.Text = "";
            // 
            // label73
            // 
            this.label73.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.label73.AutoSize = true;
            this.label73.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label73.ForeColor = System.Drawing.SystemColors.Highlight;
            this.label73.Location = new System.Drawing.Point(3, 320);
            this.label73.Name = "label73";
            this.label73.Size = new System.Drawing.Size(877, 20);
            this.label73.TabIndex = 41;
            this.label73.Text = "PROCESS UPDATE FIRMWARE";
            this.label73.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // tabControl1
            // 
            this.tabControl1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tabControl1.Controls.Add(this.tabPage3);
            this.tabControl1.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.tabControl1.Location = new System.Drawing.Point(0, 0);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(905, 544);
            this.tabControl1.TabIndex = 0;
            // 
            // timer2
            // 
            this.timer2.Interval = 300;
            this.timer2.Tick += new System.EventHandler(this.timer2_Tick);
            // 
            // openFileDialog2
            // 
            this.openFileDialog2.FileName = "openFileDialog2";
            // 
            // bgReceiverData
            // 
            this.bgReceiverData.RunWorkerCompleted += new System.ComponentModel.RunWorkerCompletedEventHandler(this.bgReceiverData_RunWorkerCompleted);
            // 
            // bgReceiverData2
            // 
            this.bgReceiverData2.DoWork += new System.ComponentModel.DoWorkEventHandler(this.bgReceiverData2_DoWork);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(907, 544);
            this.Controls.Add(this.tabControl1);
            this.MinimumSize = new System.Drawing.Size(923, 557);
            this.Name = "Form1";
            this.Text = "TCP Bootloader";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.tabPage3.ResumeLayout(false);
            this.groupBox6.ResumeLayout(false);
            this.groupBox6.PerformLayout();
            this.tableLayoutPanel3.ResumeLayout(false);
            this.tableLayoutPanel3.PerformLayout();
            this.groupBox4.ResumeLayout(false);
            this.groupBox4.PerformLayout();
            this.tableLayoutPanel2.ResumeLayout(false);
            this.tableLayoutPanel1.ResumeLayout(false);
            this.tableLayoutPanel1.PerformLayout();
            this.tabControl1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion
        private System.Windows.Forms.Timer timer1;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.OpenFileDialog openFileDialog1;
        private System.Windows.Forms.TabPage tabPage3;
        private System.Windows.Forms.Label lbTimeUpdateFirmware;
        private System.Windows.Forms.Label lbDateUpdateFirmware;
        private System.Windows.Forms.GroupBox groupBox6;
        private System.Windows.Forms.ProgressBar prgUpdateFirmwareProcess;
        private System.Windows.Forms.Button btnUpdateFirmwareProcess;
        private System.Windows.Forms.Button btnChoseFileBin;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.Label label67;
        private System.Windows.Forms.ComboBox cBoxChoseFileBin;
        private System.Windows.Forms.Label label69;
        private System.Windows.Forms.Label label70;
        private System.Windows.Forms.Timer timer2;
        private System.Windows.Forms.OpenFileDialog openFileDialog2;
        private System.Windows.Forms.Label label73;
        private System.Windows.Forms.RichTextBox rtBoxUpdateFirmwareProcess;
        private System.Windows.Forms.GroupBox groupBox4;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel1;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel3;
        private System.Windows.Forms.TableLayoutPanel tableLayoutPanel2;
        private System.ComponentModel.BackgroundWorker bgReceiverData;
        private System.ComponentModel.BackgroundWorker bgReceiverData2;
        private System.Windows.Forms.Button btnOpen;
        private System.Windows.Forms.ComboBox cBoxOpenPort;
    }
}

