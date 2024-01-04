/* MSP430G2553
 *

 * P1.3  STB    I/O口模拟SPI通讯
 * P1.4  CLK
 * P1.5  DIO
 *
 * P2.6 温度采样
 * P1.7  ADC采样电压
 * P2.2  蜂鸣器
 * P1.6  报警灯
 *
 * P2.1  PWM输出
 * P2.5  SPWM输出
 */

#include <msp430g2553.h>

#define DIO BIT5
#define CLK BIT4
#define STB BIT3

#define DS18B20_DIR P1DIR
#define DS18B20_OUT P1OUT
#define DS18B20_IN P1IN
#define DS18B20_DQ BIT2
#define DS18B20_H   DS18B20_OUT|=DS18B20_DQ        //DQ置位
#define DS18B20_L   DS18B20_OUT&=~DS18B20_DQ       //DQ复位
#define DQ_IN       DS18B20_DIR &= ~DS18B20_DQ     //设置DQ为输入
#define DQ_OUT      DS18B20_DIR |= DS18B20_DQ      //设置DQ为输出
#define READ_DQ     (DS18B20_IN&DS18B20_DQ)        //读DQ电平
#define CPU_F ((double)1000000)
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0))
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))

#define SIN_NUMA 150              //设定采样点数
unsigned int table_Valu[SIN_NUMA] = {0};                    //存放ADC10采样数据
unsigned int V1pp = 0, V2pp = 0, min = 1023, max = 0;       //采样电压峰峰值、ADC10MEM最大最小值
unsigned char page = 0, shift = 0, cnt_50ms, shift2 = 0;    //页面，确定按键等参数
unsigned int cel = 255, vol = 250, cel_set_ten = 2, cel_set_one = 7, vol_set_one = 1, vol_set_dig = 5;//显示温度与电压的参数
unsigned int dc_one = 1, dc_dig = 0;//输出PWM的参数

//共阴数码管显示代码:0~9,A,b,C,d,E,F,P,U,<space>,-
unsigned char tab[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,0x73,0x3E,0x00,0x40};
//使用公式sin_table[i]=100+100*sin(i*2*pi/360)生成的数据，小数朝零方向取整
#define cSMCLK 12000000 //定义 SMCLK为 12MHz
#define SIN_F 50 //定义输出正弦波频率为200Hz
# define SIN_NUM 360 //定义正弦波查表采样点数

const unsigned int sin_table[SIN_NUM] = {100, 101, 103, 105, 106, 108, 110, 112, 113, 115, 117, 119, 120, 122, 124, 125, 127, 129, 130, 132, 134, 135, 137, 139, 140, 142,143, 145, 146, 148, 150, 151, 152, 154, 155, 157, 158, 160, 161, 162,164, 165, 166, 168, 169, 170, 171, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186,187, 188, 189, 189, 190, 191, 192, 192, 193, 193, 194, 195, 195, 196, 196, 197, 197, 197, 198,198, 198, 199, 199, 199, 199, 199, 199, 199, 199, 200, 199, 199, 199, 199, 199,199, 199, 199, 198, 198, 198, 197, 197, 197, 196, 196, 195, 195, 194, 193, 193, 192, 192, 191, 190, 189, 189, 188, 187,186, 185, 184, 183, 182, 181, 180, 179, 178, 177, 176,175, 174, 173, 171, 170, 169, 168, 166, 165, 164, 162, 161, 160, 158, 157, 155, 154, 152, 151, 150, 148, 146, 145, 143, 142, 140, 139, 137, 135,134, 132, 130, 129, 127, 125,124, 122, 120, 119, 117, 115, 113, 112, 110, 108, 106, 105, 103, 101, 100,  98,  96,  94,  93,  91,  89,  87,  86,  84,  82,  80,  79,  77,  75,  74,  72,  70,  69,67,65,64,  62,  60,  59,  57,  56,  54,  53,  51,  49,  48,  47,  45,  44,  42,  41,  39,  38,  37,  35,  34,  33,  31,  30,  29,  28,  26,  25,  24,  23,  22,  21,  20,  19,  18,17,  16,  15,  14,13,  12,  11,  10,  10,   9,   8,   7,   7,   6,   6,   5,   4,   4,   3,   3,   2,   2,   2,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,0,   0,   0,   0,   0,   0,   0,   0,   1, 1,   1,   2,   2,   2,   3,   3,   4,   4,   5,   6,   6,   7,   7,   8,   9,  10,  10,  11,  12,  13,  14,  15,  16,  17,  18,  19,20,  21,  22,  23,  24,  25,  26,  28,  29,  30,  31,  33,  34,35,  37,  38, 39,  41,  42,  44,  45,  47,  48,  49,  51,  53,  54,  56,  57,  59,  60,  62,  64,  65,  67,  69,70,  72,  74,  75,  77,  79,  80,  82,  84,  86,  87,  89,  91,  93,  94,  96,  98};
int l=0;

void TA1_SPWM(void);
void TA0_Timer(void);
void GPIO_init(void);
void init_TM1638(void);
void Write_COM(unsigned char cmd);    //发送命令字
void TM1638_Write(unsigned char DATA);   //写TM1638
void Write_DATA(unsigned char add, unsigned char DATA);  //数码管显示
unsigned char TM1638_Read(void);   //读TM1638
unsigned char Read_key(void);      //读按键
void display_seg(void);        //界面显示
void key_detect(void);         //按键功能
void alarm(void);              //报警
void TA1_PWM(void);            //PWM初始化
void ADC10_init(void);
void ADC10_WaveSample(void);
void Write_oneLED(unsigned char num,unsigned char flag);
void Write_allLED(unsigned char LED_flag);
void DS18B20_init();                              //DS18B20初始化
unsigned char DS18B20_Reset();                    //DS18B20复位
void DS18B20_WriteData(unsigned char);            //写数据到DS18B20
unsigned char DS18B20_ReadData(void);                 //读数据
unsigned int DS18B20_Conert(void);                    //转换温度

int main(void) {
    WDTCTL = WDTPW + WDTHOLD;
//     BCSCTL1 = CALBC1_12MHZ;//设置定时器时钟为 12MHz
//     DCOCTL = CALDCO_12MHZ;
//     void Timer0_A_Init();
//     DCOCTL =DCOCTL&0xE0;//关闭混频器
//     TA1CTL|=TASSEL_2 +MC_1;
//     TA1CCTL1 |= OUTMOD_7;
//     P2SEL|= BIT1;
//     P2DIR|=BIT1;
//     P1DIR|=BIT6;
//     TA1CCR0 = 200;
//     Timer0_A_Init();
    GPIO_init();
    ADC10_init();
    P1DIR |= BIT6;
    unsigned char i;
    init_TM1638();          //初始化TM1638
    for(i = 0; i < 8; i++) {
       Write_DATA(i << 1, tab[18]);//初始化寄存器
    }
    while(1) {
        cel = DS18B20_Conert() * 0.625;   //温度数值 0.0625=xx, 0.625=xx.x, 6.25=xx.xx
        display_seg();                    //界面显示
        key_detect();                     //按键功能
        alarm();                          //报警
        ADC10_WaveSample();               //AD采样电压
        vol = (max - min)*100/325;       //电压有效值

    }

    while(1) {
        cel = DS18B20_Conert() * 0.625;   //温度数值 0.0625=xx, 0.625=xx.x, 6.25=xx.xx
        key_detect();                     //按键功能

        display_seg();                    //界面显示

        ADC10_WaveSample();               //AD采样电压

        vol = (max)*100/266;       //电压有效值

        alarm();                          //报警
        TA1CCR1 = TA1CCR0*(dc_one + 0.1 *dc_dig )/3.5;//pwm占空比
    }
}
//void Timer0_A_Init()
//{
//    TA0CCTL0 = CCIE;
//    TA0CCR0 = cSMCLK/SIN_F/SIN_NUM;
//    TA0CTL = TASSEL_2 + MC_1;
//    _EINT();
//}
//
//#pragma vector=TIMER0_A0_VECTOR
//__interrupt void Timer0_A0(void)
//{
//    _disable_interrupts();
//    TA1CCR1 = sin_table[l];
//    l++;
////   P1OUT^=BIT6; // 作为示波器的同步信号输出，便于示波器观测SPWM信号
//    if(l==360)
//    {
//        l=0;
//
//    }
//    _enable_interrupts();
//}
// /**********************************************************
// *初始化PWM
// *函数名称:TA1_PWM()
// **********************************************************/
// void TA1_PWM(void)
// {
//     BCSCTL1 = CALBC1_1MHZ;
//     DCOCTL = CALDCO_1MHZ;   //DCO校准至1MHz
//     DCOCTL =DCOCTL&0xE0;//关闭混频器
//     TA1CTL |= TASSEL_2 + ID_0 + MC_1; //时钟源SMCLK，分频系数1，增计数
//     P2DIR |= BIT1; //P2.1设为输出
//     P2SEL |= BIT1; //P2.1/TA1.1输出
// //    TA1CCTL1 = OUTMOD_7;            //由于ADC10MEM=V1/VCC*1023,V1=ADC10MEM/1023*VCC
// //    TA1CCR1 = TA1CCR0*(dc_one + 0.1 *dc_dig )/3.5; //PWM占空比duty=TA1CCR1/TA1CCR0，输出为VCC*duty
// }
// /**********************************************************
// *初始化SPWM
// *函数名称:TA1_SPWM()
// **********************************************************/
// void TA1_SPWM(void)
// {
//     TA1CCR0 = 200*3.2;  //改变来实现SPWM输出幅值的变化
// //    TA1CCTL2 = OUTMOD_7;
// //    P2SEL |= BIT4;               //TA1.2(就是p2.5口）输出sPWM
// //    P2DIR |= BIT4;
// }
// /**********************************************************
// *配置定时器A0中断
// *函数名称:TA0_Timer()
// **********************************************************/
// void TA0_Timer(void)
// {
//     TA0CTL = TASSEL_2 + MC_1 + ID_0;     //TA0时钟SMCLK，增计数，不分频12MHz
//     TA0CCTL0 = CCIE;                     //允许比较/捕获模块0的中断
//     TA0CCR0 = SMCLK / SIN_F / SIN_NUMT;  //配置合适的查表定时时间
//     _EINT();                      //开全局总中断
// }
// /**********************************************************
// *定时器中断响应函数
// *功能：PWM和SPWM输出
// **********************************************************/
// #pragma vector = TIMER0_A0_VECTOR
// __interrupt void Timer0_A0 (void)
// {
//     _disable_interrupts();
//     if(l >= SIN_NUMT)
//     {
//         l = 0;
//     }
// //    TA1CCR1 = TA1CCR0*(dc_one + 0.1 *dc_dig )/3.5; //PWM占空比duty=TA1CCR1/TA1CCR0，输出为VCC*duty
//     TA1CCR1 =(unsigned int) sin_table[l++];        // 这里进行正弦波查表后，更改占空比

//     _enable_interrupts();
// }

/**********************************************************
*ADC采样初始化
*函数名称:ADC10_init()
**********************************************************/
void ADC10_init(void)
{
    ADC10CTL0 =SREF_1 + ADC10SHT_3 + REF2_5V + REFON + REFOUT + ADC10ON;
    //基准VR+=VREF+,VR-=VSS,64×ADC10CLK采样保持时间,2.5V基准电压,基准输出打开
    ADC10CTL1 = INCH_7 + CONSEQ_0;//单通道单次转换
    ADC10AE0 |= BIT7;     //选择P1.7/A7来做ADC
}
/**********************************************************
*一个ADC采样周期，采样电压
*函数名称:ADC10_WaveSample()
**********************************************************/
void ADC10_WaveSample(void)
{
    unsigned int j = 0,i = 0;
    min = 1023;
    max = 0;
    for(j = 0; j < SIN_NUMA; j++)
    {  //采样SIN_NUMA次
        ADC10CTL0 |= ENC + ADC10SC;  //开始采样和转换
        for(i = 0;i < 300; i++); //采样延时，至少要采样一个周期的数据
        table_Valu[j] = ADC10MEM;
        if(table_Valu[j] > max)
        {
            max = table_Valu[j];
        } //找出最大值
        if(table_Valu[j] < min)
        {
            min = table_Valu[j];
        } //找出最小值
    }
}
/**********************************************************
*报警函数
*函数名称:alarm()
**********************************************************/
void alarm(void)
{
    unsigned int i=0,j=0;
    if(cel/100 > cel_set_ten)
    {
       i=1; //P2OUT |= BIT2;//温度十位大，蜂鸣器报警
    }
    else if(cel/100 == cel_set_ten && (cel/10)%10 >= cel_set_one) {//温度十位相等，个位大
        i=1;//P2OUT |= BIT2;
    }
    else {
        i=0;//未超过报警值，蜂鸣器不响
    }

    if(vol/100 > vol_set_one) {
       j=1;// P2OUT |= BIT2;//电压个位大，蜂鸣器报警
    }
else if(vol/100 == vol_set_one && (vol/10)%10 >= vol_set_dig)
{//电压个位相等，小数第一位大
       j=1;// P2OUT |= BIT2;
    }
    else {
       j=0;// P2OUT &= ~BIT1;//未超过报警值，蜂鸣器不响
    }
    if(i|j == 1)
    {
        P2OUT |= BIT2; //综合i，j报警灯亮，蜂鸣器响
        P1OUT |= BIT6;
    }
    else
    {
        P2OUT &= ~BIT2;
        P1OUT &= ~BIT6;
    }
}
/**********************************************************
*按键功能确定与实现
*函数名称:key_detect()
**********************************************************/
void key_detect(void) {
    unsigned char i;
    i = Read_key();     //读按键值
    if(i == 0)
    {   //按键0，下翻页
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        shift = 0;           //移位清零
        shift2 = 0;
        page++;       //页面值增加，0-2页分别为默认采样显示页、设置页、编号显示页
        if(page == 4)
        {
            page = 0;//超过2页，回到默认页
        }
        if(page == 1) {
            Write_allLED(0x04); //0000 0100
        }
        else if(page == 3)
            Write_allLED(0x40);
        else
            Write_allLED(0x00);
    }
    if(i == 1)
    {   //按键1，上翻页
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        shift = 0;        //移位清零
        shift2 = 0;
        if(page == 0)
        {
            page = 4;//默认页，下一个显示第2页
        }
        page--;     //页面值减少
        if(page == 1)
            Write_allLED(0x04);
        else if(page == 3)
            Write_allLED(0x40);
        else
            Write_allLED(0x00);
    }
    if(i == 2 && page == 1)
    {   //按键2，设置页，移位
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        shift++;
        if(shift == 4)
        {
            shift = 0;
        }
        if(shift2 == 2)
        {
            shift2 = 0;
        }
        switch(shift)
        {
            case 0: Write_allLED(0x04); break;
            case 1: Write_allLED(0x08); break;
            case 2: Write_allLED(0x40); break;
            case 3: Write_allLED(0x80); break;
            default: Write_allLED(0x04);break;
        }
    }
    if(i == 3 && page == 1)
    {   //按键3，设置页，数值加
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        if(shift == 0)
        {
            cel_set_ten++;//改变温度十位加
            if(cel_set_ten == 10)
            {
                cel_set_ten = 0;
            }
        }
        if(shift == 1)
        {
            cel_set_one++;//改变温度个位
            if(cel_set_one == 10)
            {
                cel_set_one = 0;
            }
        }
        if(shift == 2)
        {
           vol_set_one++; //改变电压个位
           if(vol_set_one == 10)
           {
               vol_set_one = 0;
           }
        }
        if(shift == 3)
        {
            vol_set_dig++;//改变电压小数位
            if(vol_set_dig == 10)
            {
                vol_set_dig = 0;
            }
        }
    }
    if(i == 4 && page == 1)
    {   //按键4，设置页，数值减
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        if(shift == 0)
        {
            if(cel_set_ten == 0)
            {
                cel_set_ten = 10;
            }
            cel_set_ten--;//改变温度十位加
        }
        if(shift == 1)
        {
            if(cel_set_one == 0)
            {
                cel_set_one = 10;
            }
            cel_set_one--;//改变温度个位
        }
        if(shift == 2) {
            if(vol_set_one == 0)
            {
                vol_set_one = 10;
            }
           vol_set_one--; //改变电压个位
        }
        if(shift == 3)
        {
            if(vol_set_dig == 0)
            {
                vol_set_dig = 10;
            }
            vol_set_dig--;//改变电压小数位
        }
    }
    if(i == 7)
    {
        cnt_50ms =0;
        while(Read_key() == i);
        cel = 255;
        vol = 250;
        cel_set_ten = 3;
        cel_set_one = 0;
        vol_set_one = 1;
        vol_set_dig = 5;
        dc_one = 1;
        dc_dig = 0;
    }
    if(i == 2 && page == 3)
    {      //按键2，设置页，移位
            cnt_50ms =0;
            while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
            shift2++;
            if(shift2 == 2)
            {
                shift2 = 0;
            }
            switch(shift2)
            {
                case 0: Write_allLED(0x40); break;
                case 1: Write_allLED(0x80); break;
                default: Write_allLED(0x40);break;
            }
        }
    if(i == 3 && page == 3)
    {//按键3，设置页，数值加
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        if(shift2 == 0)
        {
            dc_one++;//改变温度十位加
            if(dc_one == 10)
            {
                dc_one = 0;
            }
        }
        if(shift2 == 1) {
            dc_dig++;//改变温度个位
            if(dc_dig == 10) {
                dc_dig = 0;
            }
        }
    }
    if(i == 4 && page == 3)
    {//按键4，设置页，数值减
        cnt_50ms =0;
        while(Read_key() == i);
//等待按键释放    如果按键不释放，就继续等待，直到释放后才往下走
        if(shift2 == 0)
        {
            if(dc_one == 0)
            {
                dc_one = 10;
            }
            dc_one--;//改变温度十位加
        }
        if(shift2 == 1)
        {
            if(dc_dig== 0)
            {
                dc_dig = 10;
            }
            dc_dig--;//改变温度个位
        }
    }
}
/**********************************************************
*页面显示函数
*函数名称:display_seg()
**********************************************************/
void display_seg(void)
{
    if(page == 0)//显示界面
    {
        Write_DATA(0, tab[12]);//显示C
        Write_DATA(2, tab[cel/100]);
        Write_DATA(4, tab[(cel/10)%10]|BIT7);
        Write_DATA(6, tab[cel%10]);//温度值
        Write_DATA(8, tab[17]);//显示U
        Write_DATA(10, tab[(vol/100)]|BIT7);
        Write_DATA(12, tab[(vol/10)%10]);
        Write_DATA(14, tab[vol%10]);//电压值
    }
    if(page == 1)//设置界面
    {
        Write_DATA(0, tab[14]);
        Write_DATA(2, tab[19]);//显示E-
        Write_DATA(4, tab[cel_set_ten]);
        Write_DATA(6, tab[cel_set_one]);//温度设定值
        Write_DATA(8, tab[15]);
        Write_DATA(10, tab[19]);//显示F-
        Write_DATA(12, tab[(vol_set_one)]|BIT7);
        Write_DATA(14, tab[vol_set_dig]);//电压值
    }
    if(page == 2)//标志界面
    {
        Write_DATA(0, tab[18]);
        Write_DATA(2, tab[6]);//前两位不显示
        Write_DATA(4, tab[0]);
        Write_DATA(6, tab[5]);
        Write_DATA(8, tab[19]);
        Write_DATA(10, tab[0]);
        Write_DATA(12, tab[1]);
        Write_DATA(14, tab[18]);//605-01
    }
    if(page == 3)//PWM界面
    {
        Write_DATA(0, tab[18]);
        Write_DATA(2, tab[18]);//前两位不显示
        Write_DATA(4, tab[18]);
        Write_DATA(6, tab[18]);
        Write_DATA(8, tab[18]);
        Write_DATA(10, tab[18]);
        Write_DATA(12, tab[(dc_one)]|BIT7);
        Write_DATA(14, tab[dc_dig]);//605-01
    }
}
/**********************************************************
*初始化I/O口
*函数名称:GPIO_init()
**********************************************************/
void GPIO_init(void)
{
    P1DIR |= DIO + CLK + STB;
    P1OUT |= DIO + CLK + STB;//DIO、CLK、STB初始化为1
    P2DIR |= BIT2;//蜂鸣器
    P1OUT &= ~BIT2;
    P2DIR |= BIT3;
//    P1REN |= BIT2;
//    P1REN |= DIO + CLK + STB;
}
/**********************************************************
*TM1638初始化函数
*函数名称:init_TM1638()
**********************************************************/
void init_TM1638(void)
{
    unsigned char i;
//    P1OUT |= STB;//为了进入Write_COM(),TM1638_WRITE()后产生"下降沿"
    Write_COM(0x8a);//亮度 (0x88-0x8f)8级亮度可调 显示开，设置脉冲宽度为10/16
//    P1OUT |= STB;//写完数据后STB需置1
    Write_COM(0x40);//采用地址自动加1 普通模式，自动地址增加，写数据到显示寄存器
    P1OUT &= ~STB;// STB=3;
    TM1638_Write(0xc0);//设置起始地址 地址00H
    for(i = 0; i < 16; i++)
    {  //传送16个字节的数据,共16个单元清零(00H-15H)
        TM1638_Write(0x00);
    }
    P1OUT |= STB;//STB=3;
}
/**********************************************************
*发送命令字,调用此函数
*函数名称:Write_COM()
**********************************************************/
void Write_COM(unsigned char cmd)
{
    P1OUT &= ~STB;//STB下降沿
    TM1638_Write(cmd);
    P1OUT |= STB;//写完数据拉高STB
}
/**********************************************************
*写数据函数，1Byte
*函数名称:TM1638_Write()
**********************************************************/
void TM1638_Write(unsigned char DATA)
{
    P1DIR |= DIO;//DIO为输出
    unsigned char i;
    for(i = 0; i < 8; i++)
    {   //一个字节8位数据
        P1OUT &= ~CLK;//CLK低电平
        if(DATA & 0X01)
        {  //最低位为1
            P1OUT |= DIO;//输出高电平
        }
        else
        {  //最低位为0
            P1OUT &= ~DIO;//输出低电平
        }
        DATA >>= 1;//从低向高传输，这里右移一位
        P1OUT |= CLK;//CLK高电平
    }
}
/**********************************************************
*指定地址写入数据，数码管显示
*函数名称:Write_DATA()
**********************************************************/
void Write_DATA(unsigned char add, unsigned char DATA)
{
    Write_COM(0x44);//普通模式，固定地址，写数据到显示寄存器
    P1OUT &= ~STB;// STB=0;
    TM1638_Write(0xc0 | add);//设置地址为add
    TM1638_Write(DATA);
    P1OUT |= STB;//  STB=1;
}
/**********************************************************
*读TM1638中存储器的数据
*函数名称:TM1638_Read()
**********************************************************/
unsigned char TM1638_Read(void) {
    unsigned char i;
    unsigned char temp = 0;
    P1DIR &= ~DIO;//读数据需要DIO下降沿，设置为输入
    for(i = 0; i < 8; i++)
    {//需要读出8位数据
        temp >>= 1;//temp右移一位
        P1OUT &= ~CLK;//CLK低电平
        //if(DIO)
        if((P1IN & DIO) == DIO)
        {//如果读到DIO为1
            temp |= 0x80;//最高位置1(右移8次后即为最低位)
        }
        else
        {//如果读到DIO为0
            temp = temp & 0x7f;//最高位清0(右移8次后即为最低位)
        }
        P1OUT |= CLK;//CLK高电平
    }
    return temp;//返回temp的值
}
/**********************************************************
*读按键
*函数名称:Read_key()
**********************************************************/
unsigned char Read_key(void) {
    unsigned char c[4], i, key_value = 0;
    P1OUT &= ~STB;// STB=0;
    TM1638_Write(0x42);//读键扫数据 命令
    for(i = 0; i < 4; i++)
    {
        c[i] = TM1638_Read();
    }
    P1OUT |= STB;
//    STB=1;//4个字节数据合成一个字节
    for(i = 0; i < 4; i++) {
        key_value |= c[i] << i;
    }
    for(i = 0; i < 8; i++) {
        if((0x01 << i) == key_value)
            break;
    }
//   P1DIR |= DIO;
    if(i < 8) {
        return i;
    }
    else {
        return 8;//没有读到键值，返回8
    }
}
/**********************************************************
 *DS18B20初始化
 *函数名称:DS18B20_Init()
 *说明：本初始化程序可以不要，因为18B20在出厂时就被配置为12位精度了
 **********************************************************/
void DS18B20_Init()
{

    DS18B20_Reset();
    DS18B20_WriteData(0xCC);  // 跳过ROM
    DS18B20_WriteData(0x4E);  // 写暂存器
    DS18B20_WriteData(0x64);  // 往暂存器的第三字节中写上限值100℃
    DS18B20_WriteData(0x00);  // 往暂存器的第四字节中写下限值0℃
    DS18B20_WriteData(0x7F);  // 将配置寄存器配置为12位精度
    DS18B20_Reset();
}
/**********************************************************
 *复位DS18B20(通过存在脉冲可以判断DS1820是否损坏)
 *函数名称:DS18B20_Reset()
 **********************************************************/
unsigned char DS18B20_Reset()
{
    unsigned char flag;
    DQ_OUT;                       //DQ为输出
    DS18B20_H;
    delay_us(8);                  //延时8微秒
    DS18B20_L;                    //拉低总线
    delay_us(500);                //延时500微秒,产生复位脉冲，必须大于480微秒
    DS18B20_H;
    delay_us(80);              //15~60us 后接收 60-240us的存在脉冲，必须大于60微秒
    DQ_IN;
    if(READ_DQ)flag=0;           //等待从机 DS18B20 应答（低电平有效）
    else flag=1;
    DQ_OUT;
    delay_us(440);
    DS18B20_H;
    return(flag);
}
/**********************************************************
 *写数据到DS18B20
 *函数名称:DS18B20_WriteData()
 **********************************************************/
void DS18B20_WriteData(unsigned char wData)
{
    unsigned char i;

    DQ_OUT;                     //DQ为输出
    for (i=8;i>0;i--)
    {
        DS18B20_L;                  //拉低总线,产生写信号
        delay_us(15);               //延时在15us~30us
        if(wData&0x01)              //发送1位
        {DS18B20_H;}
        else {DS18B20_L;}
        delay_us(45);               //延时15~60us
        DS18B20_H;                  //释放总线,等待总线恢复
        wData>>=1;                  //准备下一位数据的传送
    }
}
/**********************************************************
 *从DS18B20中读出数据
 *函数名称:DS18B20_ReadData()
 **********************************************************/
unsigned char DS18B20_ReadData()
{
    unsigned char i,TmepData;

    for(i=8;i>0;i--)
    {
        TmepData>>=1;             //数据右移
        DQ_OUT;                   //DQ为输出
        DS18B20_L;                //拉低总线,产生读信号
        delay_us(6);
        DS18B20_H;                //释放总线,准备读数据
        delay_us(4);
        DQ_IN;                    //DQ为输入
        if(READ_DQ)
        {TmepData|=0x80;}
        delay_us(65);
    }
    return(TmepData);          //返回读到的数据
}
/**********************************************************
 *DS18B20转换温度
 *函数名称:DS18B20_Conert()
 **********************************************************/
unsigned int DS18B20_Conert()
{
    unsigned char TempData1,TempData2;

    DS18B20_Reset();
    DS18B20_WriteData(0xCC);       // 跳过ROM
    DS18B20_WriteData(0x44);       // 开始转换
    delay_us(500);                 //延时一般在500us不能过小
    DS18B20_Reset();
    DS18B20_WriteData(0xCC);       // 跳过ROM
    DS18B20_WriteData(0xBE);       //读取 RAM

    TempData1=DS18B20_ReadData();  //读低八位，LS Byte, RAM0
    TempData2=DS18B20_ReadData();  //读高八位，MS Byte, RAM1
    //   delay_ms(750);                 //延时750~900ms，保证工作周期
    DS18B20_Reset();
    //return (((TempData2<<8)|TempData1)*0.625); //0.0625=xx, 0.625=xx.x, 6.25=xx.xx
    return (((TempData2<<8)+TempData1)); //0.0625=xx, 0.625=xx.x, 6.25=xx.xx
}

//单独控制一个LED函数，num为需要控制的led序号，flag为0时熄灭，不为0时点亮
void Write_oneLED(unsigned char num,unsigned char flag)
{
    if(flag)
        Write_DATA(2*num+1,1);
    else
        Write_DATA(2*num+1,0);
}
 //控制全部LED函数，LED_flag表示各个LED状态
void Write_allLED(unsigned char LED_flag)
{
    unsigned char i;
    for(i=0;i<8;i++)
        {
            if(LED_flag&(1<<i))
                //Write_DATA(2*i+1,3);
                Write_DATA(2*i+1,1);
            else
                Write_DATA(2*i+1,0);
        }
}
