#include <Servo.h> // servo library  
#include <math.h> //calculo dos angulos

 Servo jo1, jo2, jo3; //juntas  

//links
float l3 = 12;
float l5 = 10.5; 

//Ponto P
float x, y, z;

//arcos 1 2 e 3
float a1,a2,a3;
float c1,c2,c3,s1,s2,s3;

//ângulos iniciais do motor servo
 float a1i = 90; //para girarmos -180 a 180 (range s1[0,360])
 float a2i = 90; //para girarmos -90 a 90 (range s2[0,180])
 float a3i = 180; // 0 a 180: arco3 0->180 ---> servo 180->0
//parametros passados ao servo(goal angles a partir dos angulos iniciais)
float pos, ga1,ga2,ga3;

 void setup()  
 {   
  Serial.begin(9600);
//Setando estado inicial
  jo1.attach(D7, 800, 2200);  // servo attach D3 pin of arduino  
  jo1.write(a1i);
  jo2.attach(D3, 800, 2200);  // servo attach D3 pin of arduino  
  jo2.write(a2i);
  jo3.attach(D4, 800, 2200);  // servo attach D3 pin of arduino  
  jo3.write(a3i);
  
  //objetivo
  x = 8;
  y = 0;
  z = 15;

  angulo1();
  angulo3(); //angulo 3 deve ser calculado antes do 2
  angulo2();

Serial.println("Angulos");
  //Serial.println(a1);
  //Serial.println(a2);
  //Serial.println(a3);

  //definindo os goal angulos de acordo com a1i a2i a3i
  ga1 = a1i+a1; 
  ga2 = a2i+a2;
  ga3 = a3i-a3;

  }  
 void loop()   
 {  
   Serial.println("Angulos");
  Serial.println(a1);
  Serial.println(a2);
  Serial.println(a3);
  //Movimentando o robô para px,py,pz (rodar j1,j2 e j3 em ga1, ga2 e ga3 respectivamente)
  delay(20*1000); //alguns seg antes de iniciar
  //junta1 - verifica se irá incrementar ou decrementar e gira 1 grau por vez
  if(a1i<ga1){
    for(pos = a1i; pos < ga1; pos += 1)
      {                                 
        jo1.write(pos); 
        delay(30);             
      }
  }else if(a1i>ga1){
    for(pos = a1i; pos >= ga1; pos-=1) 
      {                                
        jo1.write(pos);
        delay(30);         
      }
  }

  //junta2 
  if(a2i<ga2){
    for(pos = a2i; pos < ga2; pos += 1)
      {                                 
        jo2.write(pos);
        delay(30);              
      }
  }else if(a2i>ga2){
    for(pos = a2i; pos >= ga2; pos-=1) 
      {                                
        jo2.write(pos);
        delay(30);         
      }
  }

  //junta3 
  if(a3i>ga3){
    for(pos = a3i; pos >= ga3; pos -= 1)
      {                                 
        jo3.write(pos);
        delay(30);              
      }
  }else if(a3i<ga3){
    Serial.println("erro - a3 é sempre positivo e ga3 deve ser menor que a3i");
  }
  
  //depois de rotacionar, retorna à posição original após 1min
  delay(20*1000);

  //junta1
  if(a1i>ga1){
    for(pos = ga1; pos < a1i; pos += 1)
      {                                 
        jo1.write(pos);              
      }
  }else if(a1i<ga1){
    for(pos = ga1; pos >= a1i; pos-=1) 
      {                                
        jo1.write(pos);         
      }
  }

  //junta2 
  if(a2i>ga2){
    for(pos = ga2; pos < a2i; pos += 1)
      {                                 
        jo2.write(pos);
        delay(30);              
      }
  }else if(a2i<ga2){
    for(pos = ga2; pos >= a2i; pos-=1) 
      {                                
        jo2.write(pos);
        delay(30);         
      }
  }

  //junta3 
  if(a3i>ga3){
    for(pos = ga3; pos < a3i; pos += 1)
      {                                 
        jo3.write(pos);
        delay(30);              
      }
  }else if(a3i>ga3){
    Serial.println("erro");
  }

  delay(10*1000); //1min na posicao resetada

 } 

//Funções de cinemática inversa
 void angulo1(){
   //calculo do ângulo 1 em radianos usando px e py
   float a1r = atan (-x/y);
   c1 = cos(a1r);
   s1 = sin(a1r);

   //ângulo em graus
   a1 = a1r*180/PI;
 }

 void angulo3(){
   //Usando c1 e s1 para calcular o cosseno de a3
   c3 = ( pow(x,2) + pow(y,2) + pow(z,2) - pow(l3,2) - pow(l5,2) )/(2*l3*l5);
   //radianos
   float a3r = acos(c3);
   s3 = sin(a3r);

   //graus
   a3 = a3r*180/PI;
 }

 void angulo2(){
   //Funcao quadratica para descobrir x = seno do angulo
  double pa = ( pow(l5,2) * pow(s3,2) ) + pow((l3 + (l5*c3)), 2);
  float pb = (-2)*z*(l3+(l5*c3));
  float pc = pow(z,2.0) - ( pow(l5,2.0) * pow(s3,2.0) );

  float Delta = pow(pb,2) - (4*pa*pc);

  float c2_1 = (-pb+sqrt(Delta))/(2*pa);
  float c2_2 = (-pb-sqrt(Delta))/(2*pa);

  float s2_1 = ( (c2_1*(l3+ (c3*l5))) - z)/l5/s3;
  float s2_2 = ( (c2_2*(l3+ (c3*l5))) - z)/l5/s3;

  //angulo em rad
  float a2r_1 = atan2(s2_1, c2_1);
  float a2r_2 = atan2(s2_2, c2_2);
  float a2r=0;
  if ( (y>0 && x<0)|| (y>0 && x>0)){
         a2r = max(a2r_1, a2r_2);}
        else{
        a2r = min(a2r_1, a2r_2);}
  //angulo em graus
  a2 = a2r*180/PI;
 }
 