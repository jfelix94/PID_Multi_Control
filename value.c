#include "value.h"

int timerMulti, timer1, timer2;
float e1, ie1, referencia1, y;
float e2, ie2, referencia2;
float Kp[4], Ki[4], Kd[4];

void Init_valores(void){
    Kp[0] = 3;
    Kd[0] = 0;
    Ki[0] = 6;
    Kp[1] = 3;
    Kd[1] = 0;
    Ki[1] = 6;
    e1 = 0;
    ie1 = 0;
    referencia1 = 1861;

    Kp[2] = 0;
    Kd[2] = 0;
    Ki[2] = -3;
    Kp[3] = 0;
    Kd[3] = 0;
    Ki[3] = -3;
    e2 = 0;
    ie2 = 0;
    referencia2 = 2233;
    timerMulti = 500 ;
    timer1 = 50;
    timer2 = 50;
}

void Init_valores1(void){
    Kp[0] = 3;
    Kd[0] = 0;
    Ki[0] = 6;
    e1 = 0;
    ie1 = 0;
    referencia1 = 5000;

    timer1 = 50;
}

void Init_valores2(void){
    Kp[1] = 3;
    Kd[1] = 0;
    Ki[1] = 6;
    e2 = 0;
    ie2 = 0;
    referencia2 = 10000;

    timer2 = 50;
}

//----------------------
void Altera_Kp(float a, char b){
    if((b<4) && (b>=0)){
        Kp[b] = a;
    }
}

void Altera_Ki(float a, char b){
    if((b<4) && (b>=0)){
        Ki[b] = a;
    }
}

void Altera_Kd(float a, char b){
    if((b<4) && (b>=0)){
        Kd[b] = a;
    }
}

float Retorna_Kp(char b){
    if((b<4) && (b>=0)){
        return Kp[b];
    }
}

float Retorna_Ki(char b){
    if((b<4) && (b>=0)){
        return Ki[b];
    }
}

float Retorna_Kd(char b){
    if((b<4) && (b>=0)){
        return Kd[b];
    }
}
//----------------------
void Altera_timerMulti(int a){
    if(a>=0){
       timerMulti=a;
    }
}

void Altera_timer1(int a){
    if(a>=0){
       timer1=a;
    }
}

void Altera_timer2(int a){
    if(a>=0){
       timer2=a;
    }
}

void Altera_ie1(float a){
    ie1 = a;
}

void Altera_y(float a){
    y = a;
}

void Altera_e1(float a){
    e1 = a;
}

void Altera_ref1(float a){
    if(a >= 0){
        if(a<=4095){
            referencia1 = a;
        }
    }
}

//-------------------------------------

void Altera_ie2(float a){
    ie2 = a;
}

void Altera_e2(float a){
    e2 = a;
}

void Altera_ref2(float a){
    if(a >= 0){
        if(a<=4095){
            referencia2 = a;
        }
    }
}

//-------------------------

int Retorna_timerMulti(void){
    return timerMulti;
}

int Retorna_timer1(void){
    return timer1;
}

int Retorna_timer2(void){
    return timer2;
}

float Retorna_ref1(void){
    return referencia1;
}

float Retorna_e1(void){
    return e1;
}

float Retorna_y(void){
    return y;
}

float Retorna_ie1(void){
    return ie1;
}

//------------------------

float Retorna_ref2(void){
    return referencia2;
}

float Retorna_e2(void){
    return e2;
}

float Retorna_ie2(void){
    return ie2;
}



