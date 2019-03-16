from pyfirmata import Arduino, util
import os
if os.name == 'nt':
    from pynput import keyboard
import time

"""defining pins"""
#define ENB 5
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define ENA 6

ENB = 5
IN1 = 7
IN2 = 8
IN3 = 9
IN4 = 11
ENA = 6

def forward():
    """
    ORIGINAL function
    void forward(){
        digitalWrite(ENA, HIGH);    //enable L298n A channel
        digitalWrite(ENB, HIGH);    //enable L298n B channel
        digitalWrite(IN1, HIGH);    //set IN1 hight level
        digitalWrite(IN2, LOW);     //set IN2 low level
        digitalWrite(IN3, LOW);     //set IN3 low level
        digitalWrite(IN4, HIGH);    //set IN4 hight level
        Serial.println("Forward");  //send message to serial monitor
    }"""
    board.digital[ENA].write(1)
    board.digital[ENB].write(1)
    board.digital[IN1].write(1)
    board.digital[IN2].write(0)
    board.digital[IN3].write(0)
    board.digital[IN4].write(1)
def back():
    """
    void back(){
        digitalWrite(ENA, HIGH);
        digitalWrite(ENB, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        Serial.println("Back");
    }"""
    board.digital[ENA].write(1)
    board.digital[ENB].write(1)
    board.digital[IN1].write(0)
    board.digital[IN2].write(1)
    board.digital[IN3].write(1)
    board.digital[IN4].write(0)

def left():
    """
    void left(){
        digitalWrite(ENA, HIGH);
        digitalWrite(ENB, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH); 
        Serial.println("Left");
    }"""
    board.digital[ENA].write(1)
    board.digital[ENB].write(1)
    board.digital[IN1].write(0)
    board.digital[IN2].write(1)
    board.digital[IN3].write(0)
    board.digital[IN4].write(1)

def right():
    """
    void right(){
        digitalWrite(ENA, HIGH);
        digitalWrite(ENB, HIGH);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        Serial.println("Right");
    }
    """
    board.digital[ENA].write(1)
    board.digital[ENB].write(1)
    board.digital[IN1].write(1)
    board.digital[IN2].write(0)
    board.digital[IN3].write(1)
    board.digital[IN4].write(0)
def stop():
    board.digital[ENA].write(0)
    board.digital[ENB].write(0)
    board.digital[IN1].write(0)
    board.digital[IN2].write(0)
    board.digital[IN3].write(0)
    board.digital[IN4].write(0)

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
        if key.char == 'w':
            forward()
        elif key.char == 's':
            back()
        elif key.char == 'a':
            left()
        elif key.char == 'd':
            right()
        elif key.char == 'x':
            stop();
            board.exit()
            return False
    except AttributeError:
        try:
            board.exit()
        except:
            print("board not connected")
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False

#define globals
try:
    board = Arduino("COM9")
except:
    board = Arduino("/dev/ttyACM0")
iterator = util.Iterator(board)
iterator.start()
def main():
    # connect(board)
    # Collect events until released
    with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:listener.join()
    # import keyboard  # using module keyboard
    # while True:  # making a loop
    #     try:  # used try so that if user pressed other than the given key error will not be shown
    #         if keyboard.is_pressed('q'):  # if key 'q' is pressed 
    #             print('You Pressed A Key!')
    #             try:
    #                 board.exit()
    #             except:
    #                 print("not connected")
    #             break  # finishing the loop
    #         else:
    #             pass
    #     except:
    #         try:
    #             board.exit()
    #         except:
    #             print("not connected")
    #         break  # if user pressed a key other than the given key the loop will break

def main_simple():
    import time
    forward()
    time.sleep(10)
    back()
    time.sleep(10)
    stop()
if __name__ == "__main__":
    if os.name != 'nt':
        main_simple()
    else:
        main()