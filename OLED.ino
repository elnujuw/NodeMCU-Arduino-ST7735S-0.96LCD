#include "font.h"

int scl = 14;   //D5
int sda = 13;   //D7
int res = 5;    //D1
int dc = 4;     //D2
int cs = 15;    //D8

#define LCD_SCLK_CLR() digitalWrite(scl, LOW)//SCL
#define LCD_SCLK_SET() digitalWrite(scl, HIGH)

#define LCD_SDA_CLR() digitalWrite(sda, LOW)//SDA
#define LCD_SDA_SET() digitalWrite(sda, HIGH)

#define LCD_RST_CLR() digitalWrite(res, LOW)//RES
#define LCD_RST_SET() digitalWrite(res, HIGH)

#define LCD_DC_CLR()  digitalWrite(dc, LOW)//DC
#define LCD_DC_SET()  digitalWrite(dc, HIGH)
          
#define LCD_CS_CLR()  digitalWrite(cs, LOW)//CS
#define LCD_CS_SET()  digitalWrite(cs, HIGH)


#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif


//颜色
#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F  
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40 //棕色
#define BRRED            0XFC07 //棕红色
#define GRAY             0X8430 //灰色
//GUI颜色

#define DARKBLUE         0X01CF //深蓝色
#define LIGHTBLUE        0X7D7C //浅蓝色  
#define GRAYBLUE         0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN       0X841F //浅绿色
#define LGRAY            0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

short int BACK_COLOR, POINT_COLOR;


void setup()
{
  Serial.begin(115200);
  LCD_Init();
  
  LCD_Clear(BLACK);
  BACK_COLOR=BLACK;
  
  LCD_ShowString(0, 0, "NodeMCU ST7735S 0.96 LCD Driver", GREEN);
  

}

void loop()
{
}



/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：data  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t data) 
{  
  uint8_t i; 
  LCD_CS_CLR();
  for(i=0;i<8;i++)
  {       
    LCD_SCLK_CLR();
    if(data&0x80)
       LCD_SDA_SET();
    else 
       LCD_SDA_CLR();
    LCD_SCLK_SET();
    data <<= 1;   
  }   
  LCD_CS_SET();
}


/******************************************************************************
      函数说明：LCD数据写入函数
      入口数据：data 写入的8位数据
      返回值：  无
******************************************************************************/
void LCD_Write_Data8(uint8_t data)
{
  LCD_DC_SET();//写数据
  LCD_Writ_Bus(data);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：data 写入的16位数据
      返回值：  无
******************************************************************************/
void LCD_Write_Data(short int data)
{
  LCD_DC_SET();   //写数据
  LCD_Writ_Bus(data>>8);
  LCD_Writ_Bus(data);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：data 写入的8位命令
      返回值：  无
******************************************************************************/
void LCD_Write_Command(uint8_t data)
{
  LCD_DC_CLR();   //写命令
  LCD_Writ_Bus(data);
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
  if(USE_HORIZONTAL==0)
  {
    LCD_Write_Command(0x2a);    //列地址设置
    LCD_Write_Data(x1+26);
    LCD_Write_Data(x2+26);
    LCD_Write_Command(0x2b);    //行地址设置
    LCD_Write_Data(y1+1);
    LCD_Write_Data(y2+1);
    LCD_Write_Command(0x2c);    //储存器写
  }
  else if(USE_HORIZONTAL==1)
  {
    LCD_Write_Command(0x2a);    //列地址设置
    LCD_Write_Data(x1+26);
    LCD_Write_Data(x2+26);
    LCD_Write_Command(0x2b);    //行地址设置
    LCD_Write_Data(y1+1);
    LCD_Write_Data(y2+1);
    LCD_Write_Command(0x2c);    //储存器写
  }
  else if(USE_HORIZONTAL==2)
  {
    LCD_Write_Command(0x2a);    //列地址设置
    LCD_Write_Data(x1+1);
    LCD_Write_Data(x2+1);
    LCD_Write_Command(0x2b);    //行地址设置
    LCD_Write_Data(y1+26);
    LCD_Write_Data(y2+26);
    LCD_Write_Command(0x2c);    //储存器写
  }
  else
  {
    LCD_Write_Command(0x2a);    //列地址设置
    LCD_Write_Data(x1+1);
    LCD_Write_Data(x2+1);
    LCD_Write_Command(0x2b);    //行地址设置
    LCD_Write_Data(y1+26);
    LCD_Write_Data(y2+26);
    LCD_Write_Command(0x2c);    //储存器写
  }
}


//OLED的初始化
void LCD_Init(void)
{
  pinMode(scl, OUTPUT);
  pinMode(sda, OUTPUT);
  pinMode(res, OUTPUT);
  pinMode(dc, OUTPUT);
  pinMode(cs, OUTPUT);

  
  LCD_RST_SET();
  delay(100);
  LCD_RST_CLR();    //复位
  delay(200);
  LCD_RST_SET();
  delay(100);
  LCD_Write_Command(0x11); 
  delay(100);

  LCD_Write_Command(0x21); 
  
  LCD_Write_Command(0xB1); 
  LCD_Write_Data8(0x05);
  LCD_Write_Data8(0x3A);
  LCD_Write_Data8(0x3A);
  
  LCD_Write_Command(0xB2);
  LCD_Write_Data8(0x05);
  LCD_Write_Data8(0x3A);
  LCD_Write_Data8(0x3A);
  
  LCD_Write_Command(0xB3); 
  LCD_Write_Data8(0x05);  
  LCD_Write_Data8(0x3A);
  LCD_Write_Data8(0x3A);
  LCD_Write_Data8(0x05);
  LCD_Write_Data8(0x3A);
  LCD_Write_Data8(0x3A);
  
  LCD_Write_Command(0xB4);
  LCD_Write_Data8(0x03);
  
  LCD_Write_Command(0xC0);
  LCD_Write_Data8(0x62);
  LCD_Write_Data8(0x02);
  LCD_Write_Data8(0x04);
  
  LCD_Write_Command(0xC1);
  LCD_Write_Data8(0xC0);
  
  LCD_Write_Command(0xC2);
  LCD_Write_Data8(0x0D);
  LCD_Write_Data8(0x00);
  
  LCD_Write_Command(0xC3);
  LCD_Write_Data8(0x8D);
  LCD_Write_Data8(0x6A);   
  
  LCD_Write_Command(0xC4);
  LCD_Write_Data8(0x8D); 
  LCD_Write_Data8(0xEE); 
  
  LCD_Write_Command(0xC5);  /*VCOM*/
  LCD_Write_Data8(0x0E);    
  
  LCD_Write_Command(0xE0);
  LCD_Write_Data8(0x10);
  LCD_Write_Data8(0x0E);
  LCD_Write_Data8(0x02);
  LCD_Write_Data8(0x03);
  LCD_Write_Data8(0x0E);
  LCD_Write_Data8(0x07);
  LCD_Write_Data8(0x02);
  LCD_Write_Data8(0x07);
  LCD_Write_Data8(0x0A);
  LCD_Write_Data8(0x12);
  LCD_Write_Data8(0x27);
  LCD_Write_Data8(0x37);
  LCD_Write_Data8(0x00);
  LCD_Write_Data8(0x0D);
  LCD_Write_Data8(0x0E);
  LCD_Write_Data8(0x10);
  
  LCD_Write_Command(0xE1);
  LCD_Write_Data8(0x10);
  LCD_Write_Data8(0x0E);
  LCD_Write_Data8(0x03);
  LCD_Write_Data8(0x03);
  LCD_Write_Data8(0x0F);
  LCD_Write_Data8(0x06);
  LCD_Write_Data8(0x02);
  LCD_Write_Data8(0x08);
  LCD_Write_Data8(0x0A);
  LCD_Write_Data8(0x13);
  LCD_Write_Data8(0x26);
  LCD_Write_Data8(0x36);
  LCD_Write_Data8(0x00);
  LCD_Write_Data8(0x0D);
  LCD_Write_Data8(0x0E);
  LCD_Write_Data8(0x10);
  
  LCD_Write_Command(0x3A); 
  LCD_Write_Data8(0x05);
  
  LCD_Write_Command(0x36);
  if(USE_HORIZONTAL==0)LCD_Write_Data8(0x08);
  else if(USE_HORIZONTAL==1)LCD_Write_Data8(0xC8);
  else if(USE_HORIZONTAL==2)LCD_Write_Data8(0x78);
  else LCD_Write_Data8(0xA8);
  
  LCD_Write_Command(0x29); 
}


/******************************************************************************
      函数说明：LCD清屏函数
      入口数据：无
      返回值：  无
******************************************************************************/
void LCD_Clear(short int Color)
{
  short int i, j;    
  LCD_Address_Set(0, 0, LCD_W-1, LCD_H-1);
    for(i=0;i<LCD_H;i++)
    {
       for (j=0;j<LCD_W;j++)
        {
          LCD_Write_Data(Color);
        }

    }
}



/******************************************************************************
      函数说明：LCD画点函数
      入口数据：x,y   起始坐标
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(short int x, short int y, short int color)
{
  LCD_Address_Set(x, y, x, y);//设置光标位置 
  LCD_Write_Data(color);
} 


/******************************************************************************
      函数说明：LCD画大点函数
      入口数据：x,y   起始坐标
      返回值：  无
******************************************************************************/
void LCD_DrawPoint_big(short int x, short int y, short int color)
{
  LCD_Fill(x-1, y-1, x+1, y+1, color);
} 


/******************************************************************************
      函数说明：在指定区域填充颜色函数
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
      返回值：  无
******************************************************************************/
void LCD_Fill(short int xsta, short int ysta, short int xend, short int yend, short int color)
{          
  short int i,j; 
  LCD_Address_Set(xsta, ysta, xend, yend);      //设置光标位置 
  for(i=ysta;i<=yend;i++)
  {                               
    for(j=xsta;j<=xend;j++)
    {
      LCD_Write_Data(color);//设置光标位置     
    }     
  }                   
}


/******************************************************************************
      函数说明：画线函数
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
      返回值：  无
******************************************************************************/
void LCD_DrawLine(short int x1, short int y1, short int x2, short int y2, short int color)
{
  short int t; 
  int xerr=0, yerr=0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;
  delta_x = x2-x1; //计算坐标增量 
  delta_y = y2-y1;
  uRow = x1;//画线起点坐标
  uCol = y1;
  if(delta_x>0) incx = 1; //设置单步方向 
  else if (delta_x==0) incx = 0;//垂直线 
  else {incx=-1;delta_x=-delta_x;}
  if(delta_y>0) incy=1;
  else if (delta_y==0) incy=0;//水平线 
  else {incy=-1;delta_y=-delta_x;}
  if(delta_x>delta_y) distance = delta_x; //选取基本增量坐标轴 
  else distance = delta_y;
  for(t=0;t<distance+1;t++)
  {
    LCD_DrawPoint(uRow, uCol, color);//画点
    xerr += delta_x;
    yerr += delta_y;
    if(xerr>distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if(yerr>distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
}


/******************************************************************************
      函数说明：画矩形函数
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(short int x1, short int y1, short int x2, short int y2,short int color)
{
  LCD_DrawLine(x1, y1, x2, y1, color);
  LCD_DrawLine(x1, y1, x1, y2, color);
  LCD_DrawLine(x1, y2, x2, y2, color);
  LCD_DrawLine(x2, y1, x2, y2, color);
}


/******************************************************************************
      函数说明：画圆函数
      入口数据：x0,y0   圆心坐标
                r       半径
      返回值：  无
******************************************************************************/
void Draw_Circle(short int x0, short int y0, uint8_t r, short int color)
{
  int a,b;
  int di;
  a = 0;
  b = r;    
  while(a<=b)
  {
    LCD_DrawPoint(x0-b, y0-a, color);             //3           
    LCD_DrawPoint(x0+b, y0-a, color);             //0           
    LCD_DrawPoint(x0-a, y0+b, color);             //1                
    LCD_DrawPoint(x0-a, y0-b, color);             //2             
    LCD_DrawPoint(x0+b, y0+a, color);             //4               
    LCD_DrawPoint(x0+a, y0-b, color);             //5
    LCD_DrawPoint(x0+a, y0+b, color);             //6 
    LCD_DrawPoint(x0-b, y0+a, color);             //7
    a++;
    if((a*a+b*b)>(r*r))//判断要画的点是否过远
    {
      b--;
    }
  }
}


/******************************************************************************
      函数说明：显示字符函数
      入口数据：x,y    起点坐标
                num    要显示的字符
                color  颜色
      返回值：  无
******************************************************************************/
void LCD_ShowChar(short int x, short int y, uint8_t num, short int color)
{
  uint8_t pos,t,temp;
  short int x1 = x;
  if(x>LCD_W-16||y>LCD_H-16) return;     //设置窗口       
  num = num-' ';//得到偏移后的值
  LCD_Address_Set(x,y,x+8-1,y+16-1);      //设置光标位置 
  for(pos=0;pos<16;pos++)
  {
     temp = pgm_read_byte(&asc2_1608[num*16+pos]);   //调用1608字体
     for(t=0;t<8;t++)
     {
       if(temp&0x01) LCD_DrawPoint(x+t,y+pos,color);//画一个点
       else LCD_DrawPoint(x+t,y+pos,BACK_COLOR);
       temp >>= 1;
     }
  }
}


/******************************************************************************
      函数说明：显示字符串函数
      入口数据：x,y    起点坐标
                *p     字符串起始地址
      返回值：  无
******************************************************************************/
void LCD_ShowString(short int x, short int y, const char *p, short int color)
{         
    while(*p!='\0')
    {       
        if(x>LCD_W-16) {x=0;y+=16;}
        if(y>LCD_H-16) {y=x=0;LCD_Clear(POINT_COLOR);}
        LCD_ShowChar(x, y, *p, color);
        x += 8;
        p++;
    }  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(uint8_t m, uint8_t n)
{
  u32 result=1;
  while(n--) result *= m;    
  return result;
}


/******************************************************************************
      函数说明：显示整型数字函数
      入口数据：x,y    起点坐标
                num    要显示的数字
                len    要显示的数字个数
      返回值：  无
******************************************************************************/
void LCD_ShowInt(short int x, short int y, short int num, uint8_t len, short int color)
{           
  uint8_t t, temp;
  uint8_t enshow=0;
  for(t=0;t<len;t++)
  {
    temp = (num/mypow(10,len-t-1))%10;
    if(enshow==0&&t<(len-1))
    {
      if(temp==0)
      {
        LCD_ShowChar(x+8*t, y, ' ', color);
        continue;
      }else enshow=1; 
       
    }
    LCD_ShowChar(x+8*t, y, temp+48, color); 
  }
} 

/******************************************************************************
      函数说明：显示浮点数字函数
      入口数据：x,y    起点坐标
                num    要显示的数字
                len    要显示的数字个数
      返回值：  无
******************************************************************************/
void LCD_ShowFloat(short int x, short int y, float num, uint8_t len, short int color)
{           
  uint8_t t, temp;
  uint8_t enshow = 0;
  short int num1;
  num1 = num*100;
  for(t=0;t<len;t++)
  {
    temp = (num1/mypow(10,len-t-1))%10;
    if(t==(len-2))
    {
      LCD_ShowChar(x+8*(len-2), y, '.', color);
      t++;
      len += 1;
    }
    LCD_ShowChar(x+8*t, y, temp+48, color);
  }
}
