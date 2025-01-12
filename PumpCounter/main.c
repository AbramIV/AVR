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

const int arrow = '>'; //62 код ASCII для курсора (>)
int counterMS,a,flagSEC,flag,Pump,arrow_pos,out,x1,x10,x100,x1000,x10000,sym,time;
int down,up,menu,butEn,butBack,butStart,butStop,DoubleStop,xL,butRight,ms10,s,m;
int flagStop,counterErr,error,E,E1,flagE;
char buf1[1],buf10[1],buf100[1],buf1000[1],buf10000[1];
char bufc[10],bufL[8],bufMS10[2],bufSec[3],bufMin[3];
float c,V,l,x,y1,y10,y100,y1000,y10000,ms1_10,s1,m1;

ISR(TIMER0_OVF_vect)    // прерывание  таймера по переполнению
{
	ms10++;
	counterMS++;  // счетчик количества прерываний для отсчета 10 мс
	//counterErr++; // счетчик для мигания в случае ошибки запсука
	TCNT0 = 111;    // прерывание каждые 10 мс

	if (counterMS >= 100)   // 100 прерываний через каждые 10 мс = 1 с
	{
		flagSEC = 1;   // флаг отсчета секунды
		counterMS = 0; //сброс счетчика прерываний
		if (PORTB.7 == 1) PORTB.7 = 0; else PORTB.7 = 1;  // индикация работы МК
	}
}

void init_time0(void)   //настройка инициализации таймера 0
{
	TCCR0B  = 5;     // предделитель частоты таймера (1024)
	TCNT0  = 111;    // предзагрузка количества тактов в счетный регистр
	TIFR0  = 0;
	TIMSK0 = 0x01;
	EIMSK = 0;
}

void watch(void)   // часы
{
	if (ms10 >= 100)
	{
		s++;   //счетчик секунд для часов
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

void start(void)   // функция кнопки Запуска агрегата и счета оборотов
{
	if (PINC.0 == 1) flag = 0;
	if (PINC.0 == 0) flag++;
	if (flag == 1)  a++;
	c = a/7;
	V = c;
}

void stop(void)      // кнопка стоп, СТОП работает в любом режиме независимо от выбранного
{
	if (PINC.5 == 1) butStop = 0;
	{
		if (PINC.5 == 0) butStop++;
		{
			if ((butStop == 1)||(error == 1))
			{
				PORTB.1 = 0;       // индикация работы агрегата
				PORTB.2 = 0;       // остановка агрегата
				PORTB.3 = 0;       // выключение излучятеля для фотоприемника
				Pump = 0;          // обуление флага активности агрегата
				//flagStop = 1;
				if (menu == 0) time = 0;          // стоп часы
				if (menu != 0) DoubleStop++;      // счетчик двойного нажатия СТОП
				if (menu == 2) LCD_WriteCom(0x0E);   //включение курсора в режиме меню 2

				if ((DoubleStop == 2)&&(Pump == 0))   // двойное нажатие СТОП обнуляет кол-во набранных единиц
				{
					l = 0;
					DoubleStop = 0;
				}
			}
		}
	}

	if (menu == 2)     // автоматический СТОП по достижению заданного количества
	{
		if ((l>=x)||(error == 1))
		{
			PORTB.1 = 0;
			PORTB.2 = 0;
			PORTB.3 = 0;       // выключение излучятеля для фотоприемника
			Pump = 0;
		}
	}
}

void setArrow(void)       // установка курсора в меню выбора режимов
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
					LCD_WriteData(' ');   // подтирание предыдущего местоположения
					LCD_Goto(0,0);
					LCD_WriteData(arrow);
					arrow_pos = 0;        // флаг позиции курсора строки
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

void right(void)      //смещение курсора вправо  при нажатии кнопки
{
	LCD_Goto(sym,0);
	if (sym >= 12) sym = 7;
	if (PINC.7 == 1) butRight = 0;
	if (PINC.7 == 0) butRight++;
	if (butRight == 1) sym++;
}

void setLiters(void)            // установка кол-ва единиц для авто набора
{
	if ((menu == 2)&&(Pump != 1))
	{
		switch (sym)
		{
			case 7:                   // установка десятых тысяч единиц
			if (PINC.1 == 1) up = 0;
			{
				if (PINC.1 == 0) up++;
				{
					if ((up == 1)&&(x10000<9))  // увеличение на единицу при нажатии если не равно 9
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
					if ((down == 1)&&(x10000>0))   // уменьшение на единицу при нажатии если не равное 0
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

			case 8:                     // установка тысяч единиц
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

			case 9:                    // установка сотых единиц
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

			case 10:                     // установка десятых единиц
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

			case 11:                     // установка единиц
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
	if (out == 0)          // единовременно выведение информации на LCD
	{
		LCD_Goto(1,0);
		LCD_SendString("Manual");
		LCD_Goto(1,1);
		LCD_SendString("Auto");
		LCD_Goto(6,1);
		LCD_SendString("00:00:00");
		out = 1;
	}
	setArrow();               // функция курсора для меню выбора режима

	if (PINC.6 == 1) butStart = 0;
	{
		if (PINC.6 == 0) butStart++;
		{
			if (butStart == 1)           // при нажатии кнопки старт запускается секундомер
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
				switch (arrow_pos)        //  установка флага выбора режима в зависимости от положения курсора
				{
					case 0:
					menu = 1;              // флаг ручного режима
					LCD_Clear();
					break;

					case 1:
					menu = 2;               // авто режим
					out = 2;
					LCD_Clear();
					break;

					default:
					menu = 0;               // по умолчанию меню выбора режима
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
			if (butStart == 1)           // при нажатии кнопки старт запуск агрегат, установка флага запуска
			{
				Pump = 1;
				PORTB.1 = 1;                 // включение агрегата (оптопара на макете)
				PORTB.3 = 1;                 // включение излучателя для фотоприемника
				// flagStop = 0;
				DoubleStop = 0;
			}
		}
	}

	if (Pump == 1)   // если установлен флаг запуска, включение счета оборотов
	{
		start();
		PORTB.2 = 1;  // индикация движения и работы счетчика агрегата
	}

	if (flagSEC == 1)   // по истечении секунды выводи нформации на дисплей и обнуление переменных
	{
		if (c<10)
		{
			LCD_Goto(7,0);
			LCD_WriteData(' ');
			LCD_Goto(8,0);
			LCD_WriteData(' ');        // подтирание единиц измерения после уменьшении значения
			LCD_Goto(9,0);
			LCD_WriteData(' ');
			LCD_Goto(10,0);
			LCD_WriteData(' ');
		}
		LCD_Goto(0,0);
		sprintf(bufc,"v=%.0f rps",c);
		LCD_SendString(bufc);          // отображение скорости вращения в секунду
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
		LCD_SendString(bufL);          // отображение набранного количества
		a = 0;
		c = 0;
		V = 0;
		flagSEC = 0;
	}
}

void setMenu2(void)
{
	right();        // смещение курсора вправо по разрядам количества единиц  (10000 1000 100 10 1)
	setLiters();    // в зависимости от положения курсора установка определенного разряда на нужное количество (0 - 9)

	if (out == 2)       // единоразовый вывод основной информации меню 2 авто
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
			if ((butStart == 1)&&(x!=0))     // включение агрегата если нажата кнопка старт и установлено не нудевое количество единиц
			{
				Pump = 1;
				PORTB.1 = 1;                    // индикация движения агрегата и счетчика агрегата
				PORTB.3 = 1;                    // включение излучателя для фотоприемника
				DoubleStop = 0;
			}
		}
	}

	if (Pump == 1)
	{
		start();
		if (c>0) PORTB.2 = 1;  // индикация движения и работы счетчика агрегата
		LCD_WriteCom(0x0C);     // отключение курсора во время работы агрегата
	}

	xL = (x10000*10000)+(x1000*1000)+(x100*100)+(x10*10)+x1;   // расчет количества нужного количества единиц
	x = xL;                                                    // домножением на соответствующую разрядность

	if (flagSEC == 1)
	{
		if (l<10)
		{
			LCD_Goto(7,1);
			LCD_WriteData(' ');
			LCD_Goto(8,1);
			LCD_WriteData(' ');         // подтирание при уменьшении количества
			LCD_Goto(9,1);
			LCD_WriteData(' ');
			LCD_Goto(10,1);
			LCD_WriteData(' ');
		}
		l = l+V;                       // сложение результатов и вывод инфы
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

void back(void) // функция кнопки возврата в Меню выбора режима
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
				LCD_WriteCom(0x0C);  // выключение курсора
				PORTB.2 = 0;
			}
		}
	}
}

void main(void)
{
	LCD_Init(0);          // инициализация дисплея, курсор выкл
	init_time0();         // инициализация нулево таймера МК
	LCD_Clear();          // очистка дисплея
	sei();

	DDRB = 0xFF;          // порт B выход
	PORTB = 0x00;         // ноль на выходе

	DDRC = 0x00;          // порт C вход
	PORTC = 0xFF;         // подключение pull-up резисторов

	LCD_Goto(0,0);        // позиционировние курсора
	LCD_WriteData(arrow); // вывод символа вертикального курсора ">"
	arrow_pos = 0;        // установка вертикально курсора на 0 строку
	out = 0;              // сброс флага единоразового вывода меню 0
	sym = 7;              // исходная позиция горизонтального курсора в меню 2

	while(1)
	{
		stop();               // СТОП насос
		switch (menu)         // флаг выбора режима и меню
		{
			case 1:
			setMenu1();          // активно меню  Ручное
			break;

			case 2:
			setMenu2();          // активно меню Авто
			break;

			default:
			setMenu();           // по умолчанию меню Выбора режима
		}

		if (Pump != 1)         // если насос не движется
		back();            // функция возврата в Меню выбора
		if (time == 1)
		watch();
	}
}