/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools         
#include "servo.h"
#include "ping.h"

//DECLARING VARIABLES, PINS OF COMPONENTS ON PROPELLER

static int servo_right_pin = 16; 
static int servo_left_pin = 17; 
static int ir_left_pin = 12; 
static int ir_right_pin = 3;
static int dir_read_pin_left = 4; //connect with 24 of raspberry pi
static volatile int dir_read_left = 0;
static int dir_read_pin_right = 11; //connect with 25 of raspberry pi 
static volatile int dir_read_right = 0;
static int dir_reading_pin = 10; // connect with 23 of raspberry pi 
static int ir_right_outside_pin = 2;
static int ir_left_outside_pin = 13;
static int straight_pause = 450;
static int aruco_reading_pin = 6; //connect with 6 of raspberry pi
//low(aruco_reading_pin);
static int aruco_recieveing_pin = 5; //connect with 5 of raspberry pi
static volatile int aruco_recieveing = 0;
static volatile int counter_aruco = 0; 
static volatile counter_turn = 0;


static int center_path = 1; 
static int red_light_obstacle = 1;
static int intersection = 0;
static int intersection_lane_a = 1;
static int intersection_atob = 1;
static int end = 0; 

static volatile int intersection_detection = 0;  
static volatile int pickup_indicator = 0; 
static volatile int drop_indicator = 0; 
static volatile int j = 1;


int distance_counter = 0;
static int pappli = 0;
static int papplia = 0;


serial *lcd;

static int pickup_lane = 0; 
static int drop_lane = 0; 
/*lcd = serial_open(12, 12, 0, 9600);
      writeChar(lcd, ON);
      writeChar(lcd, CLR);
      writeChar(lcd, 17);*/

static int straight_speed_rightservo = -50 ;
static int straight_speed_leftservo =  50; 
static int straight_speed_rightservo_noman = -10;
static int straight_speed_leftservo_noman = 10;

static int straight_speed_laneab_rightservo = -25; 
static int straight_speed_laneab_leftservo = 25; 

static int right_turn_rightservo = 50;
static int right_turn_leftservo = 50;
static int right_turn_rightservo_noman = 10;
static int right_turn_leftservo_noman = 10;

static int left_turn_rightservo = -50;
static int left_turn_leftservo = -50;
static int left_turn_rightservo_noman = -10;
static int left_turn_leftservo_noman = -10;


static volatile int counter;
static volatile int pickup_counter = 0;

static volatile int currentstateir; 
static volatile int laststateir;
static volatile int left_aruco_detect = 0; 
static volatile int right_aruco_detect = 0;

static int trig_right = 0; 
static int echo_right = 1;
static int ping_left = 15;
static int final_counter = 0; 
static int led_pin = 8; 

static volatile int left_ir;
static volatile int right_ir;
static volatile int left_ir_outside;
static volatile int right_ir_outside; 
static volatile int last_left_ir;
static volatile int last_right_ir;
static volatile int next_lane = 0;

static volatile float distance_right = 100;
static volatile float distance_left = 100;
static volatile int intermediate_papplia = 0;
static volatile float duration;
static volatile float duration1;
static volatile int obstacle =0 ; 
static volatile int obstacle_center=0; 
static volatile int counting_final_ready = 0; 
static volatile int count_noman = 0;

void ir_status_function(void *par)
{
  while(1)
  {
    left_ir = input(ir_left_pin);
    right_ir = input(ir_right_pin);
    left_ir_outside = input(ir_left_outside_pin);
    right_ir_outside = input(ir_right_outside_pin);
  }    
} 

void intersection_detection_fn(void *par)
{
  while(1)
  {
    if(intersection_detection ==1 )
    { 
      intersection_detection = 0 ;
      high(led_pin);
      pause(1500);
      low(led_pin);

    
    }      
  }    
} 



//FUNCTION FOR DETECTING PICKUP AND DROP OFF WIDGETS
void ultrasonic_left_fn(void *par)
{
  float l1_cm;
  long duration1;
  
  while (1)
  {
    //float l_cm_avg = 0.0;
   
 /*   low(trig_left);
    pulse_out(trig_left,10);  
    long tEcho1 = pulse_in(echo_left,1);

    l1_cm = tEcho1/58.0;        */
     
    distance_left = ping_cm(ping_left);
    pause(100);
    //CHECK 
    
  }    
 
}

//FUNCTION FOR FRONT OBSTACLE DETECTION
void ultrasonic_right_fn(void *par)
{ 
  float l_cm;
  long duration;
  
  while (1)
  {
    //float l_cm_avg = 0.0;
   
    low(trig_right);
    pulse_out(trig_right,10);  
    long tEcho = pulse_in(echo_right,1);
    l_cm = tEcho/58.0;        
    
    distance_right = l_cm;
   
  }    
}


//FUNCTIONS FOR ROBOT MOTION CONTROL 
void straight()
{
  servo_speed(servo_right_pin, straight_speed_rightservo);
  servo_speed(servo_left_pin,straight_speed_leftservo);
}

void straight_noman()
{
  servo_speed(servo_right_pin, straight_speed_rightservo_noman);
  servo_speed(servo_left_pin,straight_speed_leftservo_noman);
  
}
  
void straight_lane_b()
{
  servo_speed(servo_right_pin, straight_speed_laneab_rightservo);
  servo_speed(servo_left_pin, straight_speed_laneab_leftservo);
}

void left_tilt()
{
  servo_speed(servo_right_pin, left_turn_rightservo);
  servo_speed(servo_left_pin, left_turn_leftservo);
}

void left_blind_turn_90()
{
  left_tilt();
  pause(850);    
}  
void right_tilt()
{
  servo_speed(servo_right_pin, right_turn_rightservo);
  servo_speed(servo_left_pin, right_turn_leftservo);
}

void right_tilt_noman()
{
  servo_speed(servo_right_pin, right_turn_rightservo_noman);
  servo_speed(servo_left_pin, right_turn_leftservo_noman);
}  

void left_tilt_noman()
{
  servo_speed(servo_right_pin,left_turn_rightservo_noman);
  servo_speed(servo_left_pin,left_turn_rightservo_noman);
}  

void right_blind_turn_90()
{
  right_tilt();
  pause(850);
 } 

void halt()
{
  servo_speed(servo_right_pin,0);
  servo_speed(servo_left_pin, 0); 
}  
    
    
//FUNCTION FOR LINE FOLLOWING 
void motion()
{
  if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
          
  } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
    
  }
  
  else if((left_ir == 1) && (right_ir == 1))
  {
    straight();
    pause(straight_pause);
  }           
}

//FUNCTION FOR LINE FOLLOWING in NO MAN'S LAND 
void motion_noman()
{
  if((left_ir == 0) && (right_ir == 0))
  {
   straight_noman(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
          
  } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
    
  }
  
  else if((left_ir == 1) && (right_ir == 1))
  {
    straight();
    pause(straight_pause);
  }           
}

void motion_next_lane()
{
  if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
   
    
    if(left_ir_outside == 1)
    {
      straight();
      pause(straight_pause);
      left_turn();
      halt();
      pause(200);
      next_lane = 1;    
      
    } 
    else
    {
      left_tilt();
    }
                 
          
   } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    
      if(right_ir_outside == 1)
    {
      straight();
      pause(straight_pause);
      right_turn();
      halt();
      pause(200);
      next_lane = 1;    
      
    }  
    
    else
    {
      right_tilt(); 
      } 
    
  }
  
/*  else if((left_ir == 1) && (right_ir == 1))
  {
    straight();
    pause(300);
  }   */        
}

//FUNCTION FOR LINE FOLLOWING 
void motion_ignorance_to_left()
{
  if((left_ir == 1) && (right_ir == 1) && (right_ir_outside == 1))
  {
    straight();
    pause(straight_pause);
    }
  else if((left_ir == 1) &&(left_ir_outside == 1) && (right_ir_outside == 0))
  {
    straight();
  }       
    
  
 else  if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
 else if((left_ir == 1) && (right_ir == 0))
  { 
    /*if((left_ir_outside == 1) && (right_ir_outside == 0))
    {
      printf("Straight Jane ka left Ir out");
      straight();
      
    }    
    else
    {  */
    left_tilt();
  //  }    
  } 
  
 else if((left_ir == 0) && (right_ir == 1))
  { 
   /* if((right_ir_outside == 1) && (left_ir_outside == 0))
    {
      straight();
     printf("Straight jaane ka right out ir");
      }*/
      
  //  else
   // {
      right_tilt();
     
    // }
         
   }
  
  /*else if((left_ir == 1) && (right_ir == 1))
  {

     straight();
     pause(straight_pause);
   }   */      
    
         
}

void motion_ignorance_to_right()
{
  if((left_ir == 1) && (right_ir == 1) && (left_ir_outside == 1))
  {
    straight();
    pause(straight_pause);
    }
  else if((right_ir == 1) &&(right_ir_outside == 1) && (left_ir_outside == 0))
  {
    straight();
  }       
    
  
 else  if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
 else if((left_ir == 1) && (right_ir == 0))
  { 
    /*if((left_ir_outside == 1) && (right_ir_outside == 0))
    {
      printf("Straight Jane ka left Ir out");
      straight();
      
    }    
    else
    {  */
    left_tilt();
  //  }    
  } 
  
 else if((left_ir == 0) && (right_ir == 1))
  { 
   /* if((right_ir_outside == 1) && (left_ir_outside == 0))
    {
      straight();
     printf("Straight jaane ka right out ir");
      }*/
      
  //  else
   // {
      right_tilt();
     
    // }
         
   }
  
  /*else if((left_ir == 1) && (right_ir == 1))
  {

     straight();
     pause(straight_pause);
   }   */      
    
         
}


//FUNCTION FOR LINE FOLLOWING IN LANE B DURING MANEUVER 
void motion_right_lane()
{
  /*if ((distance_right < 10) && (distance_right >0 ))
  {
    printf("right place ");
    halt(); 
    pause(1500);
    right_blind_turn_90();
    halt();
    pause(2000);
    right_aruco_detect = 1; 
  }    */
       
      
  
 if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
  } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
  }
  
  else if((left_ir == 1) && (right_ir == 1))
  { 
   halt();
   pause(1000);
   left_turn();
   right_aruco_detect = 1;
  }           
        
}


//FUNCTION FOR LINE FOLLOWING IN LANE B DURING MANEUVER 
void motion_left_lane()
{
  if (distance_left < 20)
  {
    halt(); 
    pause(1500);
    left_blind_turn_90();
    halt();
    pause(200);
    high(aruco_reading_pin);
    //pause(500);
    aruco_recieveing = input(aruco_recieveing_pin); 
    counter_aruco = 0;
    while(aruco_recieveing == 0) //make raspi pin 
    { 
       counter_aruco = counter_aruco+ 1;
       aruco_recieveing = input(aruco_recieveing_pin); 
       pause(200);
       
       if(counter_aruco>25)
       {
         break;
       }         
    }      
    left_aruco_detect = 1; 
       
  }    
  
 else if((left_ir == 0) && (right_ir == 0))
  {
   straight(); 
  }
  
  else if((left_ir == 1) && (right_ir == 0))
  {
    left_tilt();
  } 
  
  else if((left_ir == 0) && (right_ir == 1))
  {
    right_tilt();
  }
  
  else if((left_ir == 1) && (right_ir == 1))
  { 
   halt();
   pause(1000);
   right_turn();
   left_aruco_detect = 1;
  }           
      
  
}
//FUNCTION FOR LEFT TURN 
void left_turn()
{
  //int currentstateir, laststateir;
  currentstateir = left_ir;
  laststateir = currentstateir; 
  
  

  if(currentstateir == 0)
  {
    counter  = 2;
  }
  else
  {
    counter = 3; 
  }
  int i=1;

  while(i<=counter)
  { 
    left_tilt();
    currentstateir = left_ir;
    if(laststateir != currentstateir)
    {
      i++;
      laststateir = currentstateir; 
      printf("\ni value = %d", i);

    }
  }
  printf("\nout of while"); 
 
}  

//FUNCTION FOR RIGHT TURN
void right_turn()
{

  currentstateir = right_ir;
  laststateir = currentstateir; 
  
  

  if(currentstateir == 0)
  {
    counter  = 2;
  }
  else
  {
    counter = 3; 
  }
  int i=1;

  while(i<=counter)
  { 
    right_tilt();
    currentstateir = right_ir;
    if(laststateir != currentstateir)
    {
      i++;
      laststateir = currentstateir; 

    }
  }

} 

unsigned int ir_stack[40+40];
unsigned int ultrasonic_left_stack[40+40];
unsigned int ultrasonic_right_stack[40+40];
unsigned int intersection_stack[40+40];



int main()                                    // Main function
{
    cogstart(&ir_status_function, NULL,ir_stack, sizeof(ir_stack));
    cogstart(&ultrasonic_left_fn, NULL,ultrasonic_left_stack, sizeof(ultrasonic_left_stack));  
    cogstart(&ultrasonic_right_fn, NULL,ultrasonic_right_stack, sizeof(ultrasonic_right_stack));  
    cogstart(&intersection_detection_fn, NULL, intersection_stack, sizeof(intersection_stack));
 /*   while(1)
    {
      printf("Distance Right = %f\n",distance_right);
      printf("Distance Left = %f\n",distance_left);
      pause(1000);
      
      } */
    

    //pause(1000);
   low(dir_reading_pin);
    dir_read_left = 0 ;
    dir_read_right = 0;
    low(aruco_reading_pin);
    while  (j<=3)
      {
        
                  while(1)
              {
                if((left_ir == 1) && (right_ir ==1))
              {
                motion();
                break;
                }
                motion();
              }
              
              intersection_detection = 1; 
             halt();
              high(dir_reading_pin);
                printf("j===%d  \n        ",j);
             dir_read_left = 0 ;
             dir_read_right = 0;
             counter_turn = 0;
         while((dir_read_left == 0) &&(dir_read_right == 0))
          {  //counter_turn = counter_turn + 1;
          
                  printf("while 1\n");
    
            dir_read_left = input(dir_read_pin_left);
            dir_read_right = input(dir_read_pin_right);
            
                 
          }
        }          
        low(dir_reading_pin);    
      
      if(dir_read_left == 1)
        {
                printf("firat left\n");

            dir_read_left = 0;
            left_turn();
            //halt();
            //printf("After Left");
            //pause(2000);
              while(1)
            {
              if((left_ir == 1) && (right_ir ==1))
            {
              motion();
              break;
              }
              motion();
            }
            
         right_turn();
            while(1)
            {   
                  printf("in motion left LANE\n");

            motion_left_lane(); 
            if(left_aruco_detect == 1)
                { //make pin high corresponding to raspberry pi 
                  low(aruco_reading_pin);
                  left_aruco_detect = 0;
                  
                  break;
                }                  
             } 
            //ARUCO DETECTED IN LEFT, NOW GOING TO THE RIGHT LANE 
             
             left_turn();
             
            while(1)
            {
                                printf("going bXCK\n");

              if((left_ir == 1) && (right_ir ==1))
              {
                motion();
                break;
                }
              motion();
            }
            
            left_turn();
            
            //LINE FOLLOWING TO THE RIGHT LANE 
            while(1)
            {
                                printf("ON HORIZONTAL\n");

            motion_ignorance_to_right();
            if((left_ir == 1) &&(right_ir == 1)&&(left_ir_outside ==1))
              {
                motion_ignorance_to_right();
                halt();
                left_turn();
                break;
                }
            }
            
            //START DETECTION IN THE RIGHT LANE
            
            while(1)
            { 
              printf("kkjjikj\n");
               if(right_aruco_detect == 1)
              { 
                low(aruco_reading_pin);
                right_aruco_detect = 0; 
                break;  
                }
              
              if (distance_right < 20)
                {
                  printf("right place\n ");
                  halt(); 
                  pause(1500);
                  right_blind_turn_90();
                  halt();
                  pause(200);
                  high(aruco_reading_pin);
                  
                  aruco_recieveing = input(aruco_recieveing_pin);
                  counter_aruco = 0;
                  while(aruco_recieveing == 0)
                  {counter_aruco = counter_aruco + 1; 
                    aruco_recieveing = input(aruco_recieveing_pin);
                    pause(200);
                    if(counter_aruco > 25)
                    {
                      break;
                    }                      
                    
                  }                    
                  
                 right_aruco_detect = 1; 
              
                  }
              printf("distance is %f ", distance_right);
              motion_right_lane();

             
              
            } 
            
            //GOING TO THE NEXT INTERSECTION
             left_turn();
              
           while(1)
            {
              if((left_ir == 1) && (right_ir ==1))
              {
                motion();
                break;
                }
              motion();
              }
            
            
            left_turn();
              
             
                  while(1)
                {
                  motion_next_lane();
                  if(next_lane ==1)
                  { 
                    next_lane = 0;
                    halt();
                    break;
                  }                
                }
              intersection_detection = 1;   
                
            j++;
         }  
       
        
       //IF DETECTED RIGHT IN THE INTERSECTION
       
      else if(dir_read_right == 1)
      {
        right_turn();
        dir_read_right = 0;
        //halt();
        //pause(2000);
        while(1)
          {
              if((left_ir == 1) && (right_ir ==1))
            {
              motion();
               break;
            }
            motion();
          }
          
          left_turn();
          
         //ARUCO TAG DETECTION IN RIGHT SIDE  
         while(1)
          {   
               if(right_aruco_detect == 1)
              {
                low(aruco_reading_pin);
                right_aruco_detect = 0; 
                break;  
                }
              
              if (distance_right < 20)
                {
                  printf("right place\n ");
                  halt(); 
                  pause(1500);
                  right_blind_turn_90();
                  halt();
                  pause(200);
                  high(aruco_reading_pin);
                  aruco_recieveing = input(aruco_recieveing_pin);
                  counter_aruco = 0;
                  while(aruco_recieveing == 0)
                  {counter_aruco = counter_aruco + 1;
                  
                    aruco_recieveing = input(aruco_recieveing_pin);
                    pause(200);
                    if(counter_aruco >25)
                    {
                      break;
                    }                      
                    }
                  
                  
                 right_aruco_detect = 1; 
              
                  }
              printf("distance is %f ", distance_right);
              motion_right_lane();
             
         } 
         
         //DETECTED, NOW GOING BACK TO THE PREVIOUS INTERSECTION
         
         right_turn();
         
              while(1)
            {
              printf("going bXCK\n");

              if((left_ir == 1) && (right_ir ==1))
              {
                motion();
                break;
                }
              motion();
            }
            
            right_turn();
            
             //LINE FOLLOWING TO THE LEFT LANE 
            while(1)
            {
                                printf("ON HORIZONTAL\n");

            motion_ignorance_to_left();
            if((left_ir == 1) &&(right_ir == 1)&&(right_ir_outside ==1))
              {
                
                  motion_ignorance_to_left();
                  halt();
                  right_turn();
                  break;
                }
            }
            
             while(1)
            {   
                  printf("in motion left LANE\n");

            motion_left_lane(); 
            if(left_aruco_detect == 1)
                { //make pin high corresponding to raspberry pi 
                  low(aruco_reading_pin);
                  left_aruco_detect = 0;
                  break;
                }                  
             } 
             
             
             //GOING TO THE NEXT INTERSECTION
             right_turn();
             
             while(1)
            {
              if((left_ir == 1) && (right_ir ==1))
              {
                motion();
                break;
                }
              motion();
              }
              
              right_turn();
              
               while(1)
                {
                  motion_next_lane();
                  if(next_lane ==1)
                  { 
                    next_lane = 0;
                    halt();
                    break;
                  }                
                }
                j++;
                
                intersection_detection = 1; 
                    
      }      
  }   // remove "//" on this line 
  
  


//NO MAN'S LAND


    
   while((count_noman < 800))
   {     count_noman = count_noman + 1;
          printf("%d",count_noman);
          printf("\n");
         motion_noman();
    
       //  dir_read_left = input(dir_read_pin_left);
        //dir_read_right = input(dir_read_pin_right);
    
   }
   
   high(dir_reading_pin);
   dir_read_left = 0 ;
   dir_read_right = 0;
     while((dir_read_left == 0) &&(dir_read_right == 0))
      {
        //printf("while 1\n");

        dir_read_left = input(dir_read_pin_left);
        dir_read_right = input(dir_read_pin_right);
      }
      
      low(dir_reading_pin);
   
   while(1)
   {
     if(dir_read_left == 1)
     {                 
                  dir_read_left = 0;
                 high(aruco_reading_pin);
                  
                  aruco_recieveing = input(aruco_recieveing_pin);
                  while(aruco_recieveing == 0)
                  {
                    left_tilt_noman();
                    aruco_recieveing = input(aruco_recieveing_pin);
                    printf("Inside No man tilt \n"); 
                    //pause(200);
                  }
                  low(aruco_reading_pin);
                    printf("Outside No man tilt \n ");                         
                  left_tilt_noman();
                  pause(3000);
                  
            while(1)
            { 
            motion_noman();
              if((left_ir == 1) && (right_ir ==1))
              {
                halt();
                //motion();
                break;
                }
              //motion();
              }
                                         
       //tilt left slowly until aruco detected and move forward
     }    
     
     else if(dir_read_right == 1)
     {
                  dir_read_right = 0;
                  high(aruco_reading_pin);
                  
                  aruco_recieveing = input(aruco_recieveing_pin);
                  while(aruco_recieveing == 0)
                  {
                    right_tilt_noman();
                    aruco_recieveing = input(aruco_recieveing_pin);
                    //pause(200);
                  }
                  low(aruco_reading_pin);
                        
                  right_tilt_noman();
                  pause(3000);
                  
            while(1)
            {
               motion_noman();
              if((left_ir == 1) && (right_ir ==1))
              {
                //motion();
                halt();
                high(dir_reading_pin); 
                high(aruco_reading_pin);
                
                break;
                }
              //motion();
              }      
       //tilt right slowly until aruco detected
     }          
  }
 
 
  }
   
      

/*while(1)
{
  motion_ignorance();
  if((left_ir == 1) &&(right_ir == 1))
  {
    motion_ignorance();
    left_turn();
    }
  
  
  }*/

