/*
 * PumpCounter.c
 *
 * Created: 25.06.2019 12:07:45
 * Author : Abramov Igor
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <stdlib.h>
#include <lcd_lib.h>

const int arrow = '>'; //62 ��� ASCII ��� ������� (>)
int counterMS,a,flagSEC,flag,Pump,arrow_pos,out,x1,x10,x100,x1000,x10000,sym,time;
int down,up,menu,butEn,butBack,butStart,butStop,DoubleStop,xL,butRight,ms10,s,m;
int flagStop,counterErr,error,E,E1,flagE;
char buf1[1],buf10[1],buf100[1],buf1000[1],buf10000[1];
char bufc[10],bufL[8],bufMS10[2],bufSec[3],bufMin[3];
float c,V,l,x,y1,y10,y100,y1000,y10000,ms1_10,s1,m1;

ISR(TIMER0_OVF_vect)    // ����������  ������� �� ������������
{
	ms10++;
	counterMS++;  // ������� ���������� ���������� ��� ������� 10 ��
	//counterErr++; // ������� ��� ������� � ������ ������ �������
	TCNT0 = 111;    // ���������� ������ 10 ��

	if (counterMS >= 100)   // 100 ���������� ����� ������ 10 �� = 1 �
	{
		flagSEC = 1;   // ���� ������� �������
		counterMS = 0; //����� �������� ����������
		if (PORTB.7 == 1) PORTB.7 = 0; else PORTB.7 = 1;  // ��������� ������ ��
	}
}

void init_time0(void)   //��������� ������������� ������� 0
{
	TCCR0B  = 5;     // ������������ ������� ������� (1024)
	TCNT0  = 111;    // ������������ ���������� ������ � ������� �������
	TIFR0  = 0;
	TIMSK0 = 0x01;
	EIMSK = 0;
}

void watch(void)   // ����
{
	if (ms10 >= 100)
	{
		s++;   //������� ������ ��� �����
		ms10 = 0;
		LCD_Goto(12,1);
		LCD_WriteData(48);
	}

	if (s >= 60)
	{
		m++;
		s = 0;
		LCD_Goto(9,1);
		LCD_WriteData(48);
	}

	if (menu == 0)
	{
		ms1_10 = ms10;
		s1 = s;
		m1 = m;
		sprintf(bufMin,"%.0f:",m1);
		sprintf(bufSec,"%.0f:",s1);
		sprintf(bufMS10,"%.0f",ms1_10);

		if (ms10<10)
		{
			LCD_Goto(13,1);
			LCD_SendString(bufMS10);
		}
		else
		{
			LCD_Goto(12,1);
			LCD_SendString(bufMS10);
		}

		if (s<10)
		{
			LCD_Goto(10,1);
			LCD_SendString(bufSec);
		}
		else
		{
			LCD_Goto(9,1);
			LCD_SendString(bufSec);
		}

		if (m<10)
		{
			LCD_Goto(7,1);
			LCD_SendString(bufMin);
		}
		else
		{
			LCD_Goto(6,1);
			LCD_SendString(bufMin);
		}
	}
}

void start(void)   // ������� ������ ������� �������� � ����� ��������
{
	if (PINC.0 == 1) flag = 0;
	if (PINC.0 == 0) flag++;
	if (flag == 1)  a++;
	c = a/7;
	V = c;
}

void stop(void)      // ������ ����, ���� �������� � ����� ������ ���������� �� ����������
{
	if (PINC.5 == 1) butStop = 0;
	{
		if (PINC.5 == 0) butStop++;
		{
			if ((butStop == 1)||(error == 1))
			{
				PORTB.1 = 0;       // ��������� ������ ��������
				PORTB.2 = 0;       // ��������� ��������
				PORTB.3 = 0;       // ���������� ���������� ��� �������������
				Pump = 0;          // �������� ����� ���������� ��������
				//flagStop = 1;
				if (menu == 0) time = 0;          // ���� ����
				if (menu != 0) DoubleStop++;      // ������� �������� ������� ����
				if (menu == 2) LCD_WriteCom(0x0E);   //��������� ������� � ������ ���� 2

				if ((DoubleStop == 2)&&(Pump == 0))   // ������� ������� ���� �������� ���-�� ��������� ������
				{
					l = 0;
					DoubleStop = 0;
				}
			}
		}
	}

	if (menu == 2)     // �������������� ���� �� ���������� ��������� ����������
	{
		if ((l>=x)||(error == 1))
		{
			PORTB.1 = 0;
			PORTB.2 = 0;
			PORTB.3 = 0;       // ���������� ���������� ��� �������������
			Pump = 0;
		}
	}
}

void setArrow(void)       // ��������� ������� � ���� ������ �������
{
	if (menu == 0)
	{
		if (PINC.1 == 1) up = 0;
		{
			if (PINC.1 == 0) up++;
			{
				if (up == 1)
				{
					LCD_Goto(0,1);
					LCD_WriteData(' ');   // ���������� ����������� ��������������
					LCD_Goto(0,0);
					LCD_WriteData(arrow);
					arrow_pos = 0;        // ���� ������� ������� ������
				}
			}
		}

		if (PINC.2 == 1) down = 0;
		{
			if (PINC.2 == 0) down++;
			{
				if (down == 1)
				{
					LCD_Goto(0,0);
					LCD_WriteData(' ');
					LCD_Goto(0,1);
					LCD_WriteData(arrow);
					arrow_pos = 1;
				}
			}
		}
	}
}

void right(void)      //�������� ������� ������  ��� ������� ������
{
	LCD_Goto(sym,0);
	if (sym >= 12) sym = 7;
	if (PINC.7 == 1) butRight = 0;
	if (PINC.7 == 0) butRight++;
	if (butRight == 1) sym++;
}

void setLiters(void)            // ��������� ���-�� ������ ��� ���� ������
{
	if ((menu == 2)&&(Pump != 1))
	{
		switch (sym)
		{
			case 7:                   // ��������� ������� ����� ������
			if (PINC.1 == 1) up = 0;
			{
				if (PINC.1 == 0) up++;
				{
					if ((up == 1)&&(x10000<9))  // ���������� �� ������� ��� ������� ���� �� ����� 9
					{
						x10000++;
						y10000 = x10000;
						sprintf(buf10000,"%.0f",y10000);
						LCD_Goto(7,0);
						LCD_SendString(buf10000);
					}
				}
			}
			if (PINC.2 == 1) down = 0;
			{
				if (PINC.2 == 0) down++;
				{
					if ((down == 1)&&(x10000>0))   // ���������� �� ������� ��� ������� ���� �� ������ 0
					{
						x10000--;
						y10000 = x10000;
						sprintf(buf10000,"%.0f",y10000);
						LCD_Goto(7,0);
						LCD_SendString(buf10000);
					}
				}
			}
			break;

			case 8:                     // ��������� ����� ������
			if (PINC.1 == 1) up = 0;
			{
				if (PINC.1 == 0) up++;
				{
					if ((up == 1)&&(x1000<9))
					{
						x1000++;
						y1000 = x1000;
						LCD_Goto(8,0);
						sprintf(buf1000,"%.0f",y1000);
						LCD_SendString(buf1000);
						LCD_Goto(8,0);
					}
				}
			}

			if (PINC.2 == 1) down = 0;
			{
				if (PINC.2 == 0) down++;
				{
					if ((down == 1)&&(x1000>0))
					{
						x1000--;
						y1000 = x1000;
						LCD_Goto(8,0);
						sprintf(buf1000,"%.0f",y1000);
						LCD_SendString(buf1000);
						LCD_Goto(8,0);
					}
				}
			}
			break;

			case 9:                    // ��������� ����� ������
			if (PINC.1 == 1) up = 0;
			{
				if (PINC.1 == 0) up++;
				{
					if ((up == 1)&&(x100<9))
					{
						x100++;
						y100 = x100;
						LCD_Goto(9,0);
						sprintf(buf100,"%.0f",y100);
						LCD_SendString(buf100);
						LCD_Goto(9,0);
					}
				}
			}

			if (PINC.2 == 1) down = 0;
			{
				if (PINC.2 == 0) down++;
				{
					if ((down == 1)&&(x100>0))
					{
						x100--;
						y100 = x100;
						LCD_Goto(9,0);
						sprintf(buf100,"%.0f",y100);
						LCD_SendString(buf100);
						LCD_Goto(9,0);
					}
				}
			}
			break;

			case 10:                     // ��������� ������� ������
			if (PINC.1 == 1) up = 0;
			{
				if (PINC.1 == 0) up++;
				{
					if ((up == 1)&&(x10<9))
					{
						x10++;
						y10 = x10;
						LCD_Goto(10,0);
						sprintf(buf10,"%.0f",y10);
						LCD_SendString(buf10);
						LCD_Goto(10,0);
					}
				}
			}

			if (PINC.2 == 1) down = 0;
			{
				if (PINC.2 == 0) down++;
				{
					if ((down == 1)&&(x10>0))
					{
						x10--;
						y10 = x10;
						LCD_Goto(10,0);
						sprintf(buf10,"%.0f",y10);
						LCD_SendString(buf10);
						LCD_Goto(10,0);
					}
				}
			}
			break;

			case 11:                     // ��������� ������
			if (PINC.1 == 1) up = 0;
			{
				if (PINC.1 == 0) up++;
				{
					if ((up == 1)&&(x1<9))
					{
						x1++;
						y1 =  x1;
						LCD_Goto(11,0);
						sprintf(buf1,"%.0f",y1);
						LCD_SendString(buf1);
						LCD_Goto(11,0);
					}
				}
			}

			if (PINC.2 == 1) down = 0;
			{
				if (PINC.2 == 0) down++;
				{
					if ((down == 1)&&(x1>0))
					{
						x1--;
						y1 =  x1;
						LCD_Goto(11,0);
						sprintf(buf1,"%.0f",y1);
						LCD_SendString(buf1);
						LCD_Goto(11,0);
					}
				}
			}
			break;
		}
	}
}

void setMenu(void)
{
	if (out == 0)          // ������������� ��������� ���������� �� LCD
	{
		LCD_Goto(1,0);
		LCD_SendString("Manual");
		LCD_Goto(1,1);
		LCD_SendString("Auto");
		LCD_Goto(6,1);
		LCD_SendString("00:00:00");
		out = 1;
	}
	setArrow();               // ������� ������� ��� ���� ������ ������

	if (PINC.6 == 1) butStart = 0;
	{
		if (PINC.6 == 0) butStart++;
		{
			if (butStart == 1)           // ��� ������� ������ ����� ����������� ����������
			{
				time = 1;
			}
		}
	}

	if (PINC.4 == 1) butEn = 0;
	{
		if (PINC.4 == 0) butEn++;
		{
			if (butEn == 1)
			{
				switch (arrow_pos)        //  ��������� ����� ������ ������ � ����������� �� ��������� �������
				{
					case 0:
					menu = 1;              // ���� ������� ������
					LCD_Clear();
					break;

					case 1:
					menu = 2;               // ���� �����
					out = 2;
					LCD_Clear();
					break;

					default:
					menu = 0;               // �� ��������� ���� ������ ������
				}
			}
		}
	}
}

void setMenu1(void)
{
	if (PINC.6 == 1) butStart = 0;
	{
		if (PINC.6 == 0) butStart++;
		{
			if (butStart == 1)           // ��� ������� ������ ����� ������ �������, ��������� ����� �������
			{
				Pump = 1;
				PORTB.1 = 1;                 // ��������� �������� (�������� �� ������)
				PORTB.3 = 1;                 // ��������� ���������� ��� �������������
				// flagStop = 0;
				DoubleStop = 0;
			}
		}
	}

	if (Pump == 1)   // ���� ���������� ���� �������, ��������� ����� ��������
	{
		start();
		PORTB.2 = 1;  // ��������� �������� � ������ �������� ��������
	}

	if (flagSEC == 1)   // �� ��������� ������� ������ ��������� �� ������� � ��������� ����������
	{
		if (c<10)
		{
			LCD_Goto(7,0);
			LCD_WriteData(' ');
			LCD_Goto(8,0);
			LCD_WriteData(' ');        // ���������� ������ ��������� ����� ���������� ��������
			LCD_Goto(9,0);
			LCD_WriteData(' ');
			LCD_Goto(10,0);
			LCD_WriteData(' ');
		}
		LCD_Goto(0,0);
		sprintf(bufc,"v=%.0f rps",c);
		LCD_SendString(bufc);          // ����������� �������� �������� � �������
		l = l+V;

		if (l<10)
		{
			LCD_Goto(7,1);
			LCD_WriteData(' ');
			LCD_Goto(8,1);
			LCD_WriteData(' ');
			LCD_Goto(9,1);
			LCD_WriteData(' ');
			LCD_Goto(10,1);
			LCD_WriteData(' ');
		}

		LCD_Goto(0,1);
		sprintf(bufL,"L=%.1f l",l);
		LCD_SendString(bufL);          // ����������� ���������� ����������
		a = 0;
		c = 0;
		V = 0;
		flagSEC = 0;
	}
}

void setMenu2(void)
{
	right();        // �������� ������� ������ �� �������� ���������� ������  (10000 1000 100 10 1)
	setLiters();    // � ����������� �� ��������� ������� ��������� ������������� ������� �� ������ ���������� (0 - 9)

	if (out == 2)       // ������������ ����� �������� ���������� ���� 2 ����
	{
		LCD_Goto(0,0);
		LCD_SendString("Liters:");

		LCD_Goto(13,0);
		LCD_SendString("l");

		LCD_Goto(7,0);
		LCD_WriteData(48);

		LCD_Goto(8,0);
		LCD_WriteData(48);

		LCD_Goto(9,0);
		LCD_WriteData(48);

		LCD_Goto(10,0);
		LCD_WriteData(48);

		LCD_Goto(11,0);
		LCD_WriteData(48);

		LCD_WriteCom(0x0E);
		out = 3;
	}

	if (PINC.6 == 1) butStart = 0;
	{
		if (PINC.6 == 0) butStart++;
		{
			if ((butStart == 1)&&(x!=0))     // ��������� �������� ���� ������ ������ ����� � ����������� �� ������� ���������� ������
			{
				Pump = 1;
				PORTB.1 = 1;                    // ��������� �������� �������� � �������� ��������
				PORTB.3 = 1;                    // ��������� ���������� ��� �������������
				DoubleStop = 0;
			}
		}
	}

	if (Pump == 1)
	{
		start();
		if (c>0) PORTB.2 = 1;  // ��������� �������� � ������ �������� ��������
		LCD_WriteCom(0x0C);     // ���������� ������� �� ����� ������ ��������
	}

	xL = (x10000*10000)+(x1000*1000)+(x100*100)+(x10*10)+x1;   // ������ ���������� ������� ���������� ������
	x = xL;                                                    // ����������� �� ��������������� �����������

	if (flagSEC == 1)
	{
		if (l<10)
		{
			LCD_Goto(7,1);
			LCD_WriteData(' ');
			LCD_Goto(8,1);
			LCD_WriteData(' ');         // ���������� ��� ���������� ����������
			LCD_Goto(9,1);
			LCD_WriteData(' ');
			LCD_Goto(10,1);
			LCD_WriteData(' ');
		}
		l = l+V;                       // �������� ����������� � ����� ����
		LCD_Goto(0,1);
		sprintf(bufL,"L=%.1f l",l);
		LCD_SendString(bufL);

		LCD_Goto(11,1);
		sprintf(bufc,"%.0f",c);
		LCD_SendString(bufc);

		a = 0;
		c = 0;
		V = 0;
		flagSEC = 0;
	}
}

void back(void) // ������� ������ �������� � ���� ������ ������
{
	if (PINC.3 == 1) butBack = 0;
	{
		if (PINC.3 == 0) butBack++;
		{
			if (butBack == 1)
			{
				x1 = 0;
				x10 = 0;
				x100 = 0;
				x1000 = 0;
				x10000 = 0;
				sym = 7;
				out = 0;
				menu = 0;
				DoubleStop = 0;
				arrow_pos = 0;
				LCD_Clear();
				LCD_Goto(0,0);
				LCD_WriteData(arrow);
				LCD_WriteCom(0x0C);  // ���������� �������
				PORTB.2 = 0;
			}
		}
	}
}

void main(void)
{
	LCD_Init(0);          // ������������� �������, ������ ����
	init_time0();         // ������������� ������ ������� ��
	LCD_Clear();          // ������� �������
	sei();

	DDRB = 0xFF;          // ���� B �����
	PORTB = 0x00;         // ���� �� ������

	DDRC = 0x00;          // ���� C ����
	PORTC = 0xFF;         // ����������� pull-up ����������

	LCD_Goto(0,0);        // ��������������� �������
	LCD_WriteData(arrow); // ����� ������� ������������� ������� ">"
	arrow_pos = 0;        // ��������� ����������� ������� �� 0 ������
	out = 0;              // ����� ����� ������������� ������ ���� 0
	sym = 7;              // �������� ������� ��������������� ������� � ���� 2

	while(1)
	{
		stop();               // ���� �����
		switch (menu)         // ���� ������ ������ � ����
		{
			case 1:
			setMenu1();          // ������� ����  ������
			break;

			case 2:
			setMenu2();          // ������� ���� ����
			break;

			default:
			setMenu();           // �� ��������� ���� ������ ������
		}

		if (Pump != 1)         // ���� ����� �� ��������
		back();            // ������� �������� � ���� ������
		if (time == 1)
		watch();
	}
}