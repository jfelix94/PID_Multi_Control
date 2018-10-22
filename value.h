/*
 * value.h
 *
 *  Created on: 23 de jul de 2018
 *      Author: jfeli
 */

#ifndef VALUE_H_
#define VALUE_H_

        void Init_valores(void);
        void Init_valores1(void);
        void Init_valores2(void);

        void Altera_Kp(float a, char b);
        void Altera_Kd(float a, char b);
        void Altera_Ki(float a, char b);

        float Retorna_Kd(char b);
        float Retorna_Ki(char b);
        float Retorna_Kp(char b);

        void Altera_e1(float a);
        void Altera_ie1(float a);
        void Altera_timerMulti(int a);
        void Altera_timer1(int a);
        void Altera_timer2(int a);
        void Altera_ref1(float a);
        void Altera_y(float a);

        void Altera_e2(float a);
        void Altera_ie2(float a);
        void Altera_ref2(float a);

        int   Retorna_timerMulti(void);
        int   Retorna_timer1(void);
        int   Retorna_timer2(void);
        float Retorna_ref1(void);
        float Retorna_e1(void);
        float Retorna_ie1(void);
        float Retorna_y(void);

        float Retorna_ref2(void);
        float Retorna_e2(void);
        float Retorna_ie2(void);

#endif /* VALUE_H_ */
