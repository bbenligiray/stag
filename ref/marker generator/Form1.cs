using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Drawing.Imaging;


namespace EDMarker_Generator
{

    public unsafe partial class Form1 : Form
    {
        // number of bits in a marker
        int noOfBits = 48;

        // following values are in ratios, where a marker edge length is 1

        // border width
        float border = 0.125f;
        
        // radius of the circle border 
        float outerCircleRadius = 0.4f;
        // radius of the circle that encloses code bits
        float innerCircleRadius = 0.35f;
        // radius of the code circles (in ratio to innerCircleRadius)
        float codeRadius = 0.062482177287080f;
        // radius of the filler code circles (in ratio to codeRadius)
        float fillerCodeRadius = 0.7f;

        // following values are in pixels

        int fileSize = 1000;

        float markerSize;
        float borderSize;

        float outerCircleDiameterSize;
        float innerCircleDiameterSize;
        float outerCircleTopLeft;
        float innerCircleTopLeft;
        float codeCircleDiameterSize;
        float fillerCircleDiameterSize;


        // code related stuff
        
        int HD;
        List<List<Byte>> codes;
        List<doublePoint> codeLocs;
        List<List<int>> nearbyCodes;


        Font largeFont, smallFont;
        Pen grayPen;
        
        

        public Form1()
        {
            InitializeComponent();

            markerSize = fileSize / (1 + border * 2);
            borderSize = markerSize * border;

            outerCircleDiameterSize = 2 * markerSize * outerCircleRadius;
            innerCircleDiameterSize = 2 * markerSize * innerCircleRadius;
            outerCircleTopLeft = (fileSize - outerCircleDiameterSize) / 2;
            innerCircleTopLeft = (fileSize - innerCircleDiameterSize) / 2;
            codeCircleDiameterSize = 2 * innerCircleDiameterSize * codeRadius;
            fillerCircleDiameterSize = codeCircleDiameterSize * fillerCodeRadius;

            largeFont = new Font("Century Gothic", 72);
            smallFont = new Font("Century Gothic", 20);
            grayPen = new Pen(Color.Gray, 3);

            fillLocs();

            for (HD = 11; HD <= 23; HD += 2)
            {
                readCodeList();
                drawMarkers();
            }
                

        }

        public void fillLocs()
        {
            codeLocs = new List<doublePoint>();

            for (int i = 0; i < 4; i++)
            {
                codeLocs.Add(polarToCart(0.088363142525988, 0.785398163397448 + i * (Math.PI / 2)));

                codeLocs.Add(polarToCart(0.206935928182607, 0.459275804122858 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.206935928182607, (Math.PI / 2) - 0.459275804122858 + i * (Math.PI / 2)));

                codeLocs.Add(polarToCart(0.313672146827381, 0.200579720495241 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.327493143484516, 0.591687617505840 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.327493143484516, (Math.PI / 2) - 0.591687617505840 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.313672146827381, (Math.PI / 2) - 0.200579720495241 + i * (Math.PI / 2)));

                codeLocs.Add(polarToCart(0.437421957035861, 0.145724938287167 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.437226762361658, 0.433363129825345 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.430628029742607, 0.785398163397448 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.437226762361658, (Math.PI / 2) - 0.433363129825345 + i * (Math.PI / 2)));
                codeLocs.Add(polarToCart(0.437421957035861, (Math.PI / 2) - 0.145724938287167 + i * (Math.PI / 2)));
            }

            nearbyCodes = new List<List<int>>();

            for (int i = 0; i < noOfBits; i++)
            {
                nearbyCodes.Add(new List<int>());

                for (int j = 0; j < noOfBits; j++)
                {
                    if (i == j)
                        continue;
                    if (distanceBetweenDoublePoints(codeLocs[i], codeLocs[j]) < codeRadius * 4)
                        nearbyCodes[i].Add(j);
                }
            }
        }

        public void drawMarkers()
        {
            String dirName = "HD" + HD.ToString();
            System.IO.Directory.CreateDirectory(dirName);

            Bitmap img = new Bitmap(fileSize, fileSize);
            Graphics g = Graphics.FromImage(img);
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.AntiAlias;
            
            for (int i = 0; i < codes.Count; i++)
            {
                // we are working with the same bitmap, so clear it
                g.Clear(Color.White);

                // draw the outer rectangle
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.None;
                g.FillRectangle(Brushes.Black, borderSize, borderSize, markerSize, markerSize);
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

                // draw the outer circle
                g.FillEllipse(Brushes.White, outerCircleTopLeft, outerCircleTopLeft, outerCircleDiameterSize, outerCircleDiameterSize);

                // turn off antialiasing to apply morphological operations
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.None;

                // apply black code circles
                for (int j = 0; j < noOfBits; j++)
                    if (codes[i][j] == 1)
                        g.FillEllipse(Brushes.Black, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].x - codeCircleDiameterSize / 2, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].y - codeCircleDiameterSize / 2, codeCircleDiameterSize, codeCircleDiameterSize);
                // apply filler circles
                for (int j = 0; j < noOfBits; j++)
                    for (int k = j + 1; k < noOfBits; k++)
                        if ((codes[i][j] == 1) && (codes[i][k] == 1))
                            if (nearbyCodes[j].Contains(k))
                                g.FillEllipse(Brushes.Black, innerCircleTopLeft + innerCircleDiameterSize * (float)((codeLocs[j].x + codeLocs[k].x) / 2) - fillerCircleDiameterSize / 2, innerCircleTopLeft + innerCircleDiameterSize * (float)((codeLocs[j].y + codeLocs[k].y) / 2) - fillerCircleDiameterSize / 2, fillerCircleDiameterSize, fillerCircleDiameterSize);
                // apply white code circles
                for (int j = 0; j < noOfBits; j++)
                    if (codes[i][j] == 0)
                        g.FillEllipse(Brushes.White, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].x - codeCircleDiameterSize / 2, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].y - codeCircleDiameterSize / 2, codeCircleDiameterSize, codeCircleDiameterSize);
   
                //erode, dilate
                int rad = 12;
                for (int j = 0; j < 5; j++)
                {
                    dilateBitmap(ref img, 5, rad);
                    erodeBitmap(ref img, 5, rad);
                }
                
                // smooth and recode
                smoothBitmap(ref img);
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                // apply black code circles
                for (int j = 0; j < noOfBits; j++)
                    if (codes[i][j] == 1)
                        g.FillEllipse(Brushes.Black, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].x - codeCircleDiameterSize / 2, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].y - codeCircleDiameterSize / 2, codeCircleDiameterSize, codeCircleDiameterSize);
                // apply white code circles
                for (int j = 0; j < noOfBits; j++)
                    if (codes[i][j] == 0)
                        g.FillEllipse(Brushes.White, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].x - codeCircleDiameterSize / 2, innerCircleTopLeft + innerCircleDiameterSize * (float)codeLocs[j].y - codeCircleDiameterSize / 2, codeCircleDiameterSize, codeCircleDiameterSize);

                // clear the space between the inner and outer circle
                Bitmap ringImg = new Bitmap(fileSize, fileSize);
                Graphics gRing = Graphics.FromImage(ringImg);
                gRing.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                gRing.FillRectangle(Brushes.Black, 0, 0, fileSize, fileSize);
                gRing.FillEllipse(Brushes.White, outerCircleTopLeft, outerCircleTopLeft, outerCircleDiameterSize, outerCircleDiameterSize);
                gRing.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.None;
                gRing.FillEllipse(Brushes.Black, innerCircleTopLeft - 1, innerCircleTopLeft - 1, innerCircleDiameterSize + 2, innerCircleDiameterSize + 2);
                gRing.Dispose();
                ringImg.MakeTransparent(Color.Black);
                g.DrawImage(ringImg, 0, 0);

                // In case we want crosshair
                /*
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.None;
                g.FillEllipse(Brushes.White, (float)(outputSize / 2 - 8), (float)(outputSize / 2 - 8), 16, 16);
                g.DrawLine(pen, (float)(outputSize / 2 - 7), (float)(outputSize / 2 ), (float)(outputSize / 2 + 8), (float)(outputSize / 2 ));
                g.DrawLine(pen, (float)(outputSize / 2), (float)(outputSize / 2 - 7 ), (float)(outputSize / 2), (float)(outputSize / 2 + 8 ));
                g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.HighQuality;
                */

                // write the strings
                SizeF sz = g.VisibleClipBounds.Size;
                float centerString = (float)(2.375 * borderSize);
                g.TranslateTransform(centerString, centerString);
                g.RotateTransform(315);
                sz = g.MeasureString(createIndexString(i), largeFont);
                g.DrawString(createIndexString(i), largeFont, Brushes.White, -(sz.Width / 2), -(sz.Height / 2));
                g.ResetTransform();

                centerString = (float)(1.875 * borderSize);
                g.TranslateTransform(centerString, centerString);
                g.RotateTransform(315);
                sz = g.MeasureString("HD" + HD.ToString(), smallFont);
                g.DrawString("HD" + HD.ToString(), smallFont, Brushes.White, -(sz.Width / 2), -(sz.Height / 2));
                g.ResetTransform();

                // save image
                String idStr = createIndexString(i);
                while (idStr.Length < 5)
                    idStr = "0" + idStr;
                img.Save(dirName + "/" + idStr + ".png", System.Drawing.Imaging.ImageFormat.Png);
            }
        }

        public double distanceBetweenDoublePoints(doublePoint p1, doublePoint p2)
        {
            return Math.Sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        }

        public String createIndexString(int index)
        {
            String s = index.ToString();

            while (s.Length != codes.Count.ToString().Length)
                s = "0" + s;

            return s;
        }

        public doublePoint polarToCart(double radius, double radians)
        {
            return new doublePoint(0.5 + Math.Cos(radians) * radius, 0.5 - Math.Sin(radians) * radius);
        }

        public void erodeBitmap(ref Bitmap b, int thres, int radius)
        {
            Rectangle rect = new Rectangle(0, 0, fileSize, fileSize);
            BitmapData imgData = b.LockBits(rect, ImageLockMode.WriteOnly, b.PixelFormat);


            Bitmap refImage = (Bitmap)b.Clone();
            BitmapData refData = refImage.LockBits(rect, ImageLockMode.ReadOnly, refImage.PixelFormat);

            List<Point> list = generateBallMask(radius);

            int border = (int)outerCircleTopLeft;

            for (int j = border; j < fileSize - border; j++)
            {
                for (int k = border; k < fileSize - border; k++)
                {
                    if (!isWhite(refData, j, k))
                    {
                        int tot = countNonPixels(refData, j, k, true, list);
                      

                        if (tot > list.Count / 2)
                        {
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4)) = 255;
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4) + 1) = 255;
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4) + 2) = 255;
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4) + 3) = 255;
                        }
                    }
                }
            }
            refImage.UnlockBits(refData);
            refImage.Dispose();
            b.UnlockBits(imgData);
        }

        public void dilateBitmap(ref Bitmap b, int thres, int radius)
        {
            Rectangle rect = new Rectangle(0, 0, fileSize, fileSize);
            BitmapData imgData = b.LockBits(rect, ImageLockMode.WriteOnly, b.PixelFormat);


            Bitmap refImage = (Bitmap)b.Clone();
            BitmapData refData = refImage.LockBits(rect, ImageLockMode.ReadOnly, refImage.PixelFormat);

            List<Point> list = generateBallMask(radius);

            int border = (int)outerCircleTopLeft;

            for (int j = border; j < fileSize - border; j++)
            {
                for (int k = border; k < fileSize - border; k++)
                {
                    if (!isBlack(refData, j, k))
                    {
                        int tot = countNonPixels(refData, j, k, false, list);

                        if (tot > list.Count / 2)
                        {
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4)) = 0;
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4) + 1) = 0;
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4) + 2) = 0;
                            *((byte*)(imgData.Scan0 + k * imgData.Stride + j * 4) + 3) = 255;
                        }
                    }
                }
            }
            refImage.UnlockBits(refData);
            refImage.Dispose();
            b.UnlockBits(imgData);
        }

        public void smoothBitmap(ref Bitmap b)
        {

            Rectangle rect = new Rectangle(0, 0, fileSize, fileSize);
            BitmapData imgData = b.LockBits(rect, ImageLockMode.WriteOnly, b.PixelFormat);


            Bitmap refImage = (Bitmap)b.Clone();
            BitmapData refData = refImage.LockBits(rect, ImageLockMode.ReadOnly, refImage.PixelFormat);

            for (int i = 0; i < b.Width; i++)
            {
                for (int j = 0; j < b.Height; j++)
                {
                    if (Math.Sqrt((i - b.Width / 2) * (i - b.Width / 2) + (j - b.Height / 2) * (j - b.Height / 2)) < b.Width / 2 - outerCircleTopLeft - 10)
                    {
                        if (isBlack(refData, i, j))
                        {
                            bool hasWhite = false;
                            int total = 0;
                            int maskRad = 1;
                            for (int n1 = -maskRad; n1 < maskRad + 1; n1++)
                            {
                                for (int n2 = -maskRad; n2 < maskRad + 1; n2++)
                                {
                                    if ((isWhite(refData, i + n1, j + n2)) && (Math.Abs(n1 + n2) <2))
                                        hasWhite = true;


                                    total += readPix(refData, i + n1, j + n2); 
                                }
                            }
                            if (hasWhite)
                            {
                                *((byte*)(imgData.Scan0 + j * imgData.Stride + i * 4)) = (byte)((double)total / ((maskRad * 2 + 1) * (maskRad * 2 + 1)));
                                *((byte*)(imgData.Scan0 + j * imgData.Stride + i * 4) + 1) = (byte)((double)total / ((maskRad * 2 + 1) * (maskRad * 2 + 1)));
                                *((byte*)(imgData.Scan0 + j * imgData.Stride + i * 4) + 2) = (byte)((double)total / ((maskRad * 2 + 1) * (maskRad * 2 + 1)));
                                *((byte*)(imgData.Scan0 + j * imgData.Stride + i * 4) + 3) = 255;
                            }
                        }
                    }
                }
            }


                b.UnlockBits(imgData);
                refImage.UnlockBits(refData);
                refImage.Dispose();
        }

        public int countNonPixels(BitmapData bd, int x, int y, bool nonBlack, List<Point> list)
        {
            int tot = 0;


            for (int i = 0; i < list.Count; i++)
            {
                if (nonBlack) // count non-black pixels
                {
                    if (!isBlack(bd, x + list[i].X, y + list[i].Y))
                        tot++;
                }
                else
                {
                    if (!isWhite(bd, x + list[i].X, y + list[i].Y))
                        tot++;
                }
            }


                return tot;
        }

        public List<Point> generateBallMask(int r)
        {
            List<Point> maskList = new List<Point>();

            for (int i = -r; i <= r; i++)
            {
                for (int j = -r; j <= r; j++)
                {
                    if ((i == 0) && (j == 0))
                        continue;
                    if (i * i + j * j <= r * r)
                        maskList.Add(new Point(i, j));
                }
            }

                return maskList;
        }

        //BGRA
        public bool isWhite(BitmapData bd, int x, int y)
        {
            byte* b = (byte*)bd.Scan0 + y * bd.Stride + x * 4;

            if ((*(b) != 255) || (*(b + 1) != 255) || (*(b + 2) != 255))
                return false;
            else
                return true;
        }
        public bool isBlack(BitmapData bd, int x, int y)
        {
            byte* b = (byte*)bd.Scan0 + y * bd.Stride + x * 4;

            if ((*(b) != 0) || (*(b + 1) != 0) || (*(b + 2) != 0))
                return false;
            else
                return true;
        }
        public int readPix(BitmapData bd, int x, int y)
        {
            byte* b = (byte*)bd.Scan0 + y * bd.Stride + x * 4;

            return (int)(*(b));
        }
        public void readCodeList()
        {
            codes = new List<List<byte>>();
            StreamReader sr = new StreamReader("HD" + HD.ToString() + ".txt");

            while(true)
            {
                String s = sr.ReadLine();
                if (s == null)
                {
                    break;
                }
                    
                List<byte> line = new List<byte>();
                for (int j = 0; j < noOfBits; j++)
                {
                    line.Add((byte)(Convert.ToInt32(s[j]) - 48));
                }
                codes.Add(line);
            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }
    }

    public class doublePoint
    {
        public double x, y;
        public doublePoint(double inpX, double inpY)
        {
            x = inpX;
            y = inpY;
        }
    };
}
