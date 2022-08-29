/*
 * フォトリフレクタで車両の位置を検知してエンドレス線路で２列車同時運転をしてみよう！
 * 2022/9/1 7M4MON
 * 
 * Article:
 * http://nomulabo.com/n_train_auto_control/
 * 
 * スライドスイッチで1列車のみor2列車同時運転かを選択します。
 * 電源投入時は列車の制御は停止状態となっています。
 * 停車位置のセンサが車両を検出すると緑信号に変わりますので、半固定抵抗を回して閾値を調整します。
 * 調整が終わったら、タクトスイッチを押下し、列車の運転を開始します。
 * 列車が駅構内に入ったことを検出すると、黄色信号となり、Duty比を徐行状態まで指定のステップで落として減速します。
 * 列車が停止位置に到達したことを検出すると、列車を停止します。
 * 2列車同時運転の場合は、対向ホームに列車が到着するのを待ちます。
 * 発車条件を満たしたら、発車ブザーを鳴らした後、列車を一定時間（半固定抵抗で調整可）巡航速度にし、次の区間に渡します。
 * 
 */

#include <MsTimer2.h>

#define PIN_MOTOR_A 10          //PWM 9 or 10
#define PIN_SENS_STOP_A A2      //ANALOG
#define PIN_SENS_SLOW_A A3      //ANALOG
#define PIN_LED_RED_A 12
#define PIN_LED_GREEN_A 11     

#define PIN_MOTOR_B 9           //PWM 9 or 10
#define PIN_SENS_STOP_B A0      //ANALOG
#define PIN_SENS_SLOW_B A1      //ANALOG
#define PIN_LED_RED_B 8
#define PIN_LED_GREEN_B 7

#define PIN_BUZZER 3            // Tone 11 or 3
#define PIN_DUO_SEL 2           // スライドスイッチ
#define PIN_BUTTON_INIT 4       // タクトスイッチ
#define PIN_ONB_LED 13

#define PIN_SENS_MIN_DUTY A5     // 20%程度
#define PIN_SENS_MAX_DUTY A4     // 40%程度
#define PIN_VOL_THRESHOLD A6     // 3V程度
#define PIN_VOL_WAITTIME A7      // 半分=5.12秒程度

/*
#define REFRECTOR_THRESHOLD 512     // 半固定抵抗から読むように変更
#define MINIMUM_PWM_DUTY 33
#define MAXIMUM_PWM_DUTY 66
*/

typedef struct {
    uint8_t state;
    uint8_t sens_stop;
    uint8_t sens_slow;
    int16_t pwm_duty;
    uint8_t last_pwm_duty;
    uint8_t pin_pwm_out;        //なんとピンも変数でできる。
    uint8_t pin_led_red;
    uint8_t pin_led_green;
} Control_t;

Control_t control_a, control_b;

uint16_t maximum_pwm_duty, minimum_pwm_duty, refrector_threshold;

#define STATE_TRAIN_WAIT 0      //駅に来るのを待つ
#define STATE_TRAIN_ENTRY 1     //駅構内で減速
#define STATE_TRAIN_STOP 2      //駅到着
#define STATE_INIT 9

#define SLOW_DOWN_STEP 5

/* 
* アナログ入力がハイインピーダンスだと前の影響を受けて正しい値が出ないので
* 一回読んで捨ててちょっと待ってもう一度読む。 
* 入力ピンのキャパシタである程度救えているが、本来はバッファを挿入するべき。
*/
uint16_t analogReadHiz(uint8_t pin_analog_in){
    analogRead(pin_analog_in);
    delay(2);
    return analogRead(pin_analog_in);
}

/* 電源投入後に構造体を初期化する。 */
void init_train_control(Control_t* control){
    pinMode(control->pin_led_red, OUTPUT);
    pinMode(control->pin_led_green, OUTPUT);
    pinMode(control->pin_pwm_out, OUTPUT);
    digitalWrite(control->pin_led_red, HIGH);         // 黄色信号
    digitalWrite(control->pin_led_green, HIGH);       // 黄色信号
    analogWrite(control->pin_pwm_out, 0);
    control->state = STATE_INIT;
    control->last_pwm_duty = 0;
    control->pwm_duty = 0;
}

/* 駅から出発した時の状態 */
void reset_train_control(Control_t* control){
    control->state = STATE_TRAIN_WAIT;
    control->sens_stop = false;
    control->sens_slow = false;
    control->pwm_duty = analogReadHiz(PIN_SENS_MAX_DUTY) >> 2;
    analogWrite(control->pin_pwm_out ,control->pwm_duty);
}

/* センサーを読む。*/
void check_sensor(){
    uint16_t refrector_threshold = analogReadHiz(PIN_VOL_THRESHOLD);
    control_a.sens_stop = (analogReadHiz(PIN_SENS_STOP_A) < refrector_threshold) ? true : false;
    control_a.sens_slow = (analogReadHiz(PIN_SENS_SLOW_A) < refrector_threshold) ? true : false;
    control_b.sens_stop = (analogReadHiz(PIN_SENS_STOP_B) < refrector_threshold) ? true : false;
    control_b.sens_slow = (analogReadHiz(PIN_SENS_SLOW_B) < refrector_threshold) ? true : false;
}

/* 列車が駅から出発してよいかの判断
* ２列車運転なら２ホーム停車するまで待つ。１編成のみなら１つ感知でtrue
*/
uint8_t check_ready_go(){
    uint8_t state_ready_go = false;
    if (digitalRead(PIN_DUO_SEL)){  //2編成制御のとき
        if(control_a.state == STATE_TRAIN_STOP && control_b.state == STATE_TRAIN_STOP ){    // 両方のホームに来るのを待つ
            state_ready_go = true;
        }
    }else{                          //1編成制御のとき
        if(control_a.state == STATE_TRAIN_STOP || control_b.state == STATE_TRAIN_STOP ){    // どちらかのホームに来たら発車
            state_ready_go = true;
        }
    }
    return state_ready_go;
}

/* 停車後、１秒待ってブザーを鳴らし、一定時間加速して次の区間に列車を渡す */
void train_go(){
    delay(1000);
    // 発車ブザーを鳴らす。
    tone(PIN_BUZZER, 1000);
    delay(500);
    noTone(PIN_BUZZER);
    reset_train_control(&control_a);
    reset_train_control(&control_b);
    uint16_t wait_time_to_next_section = analogReadHiz(PIN_VOL_WAITTIME) + 1;  //10~10240ms
    while (wait_time_to_next_section > 1){
        delay(10);    // 次の区間に列車を渡す
        wait_time_to_next_section--;
    }
    digitalWrite(PIN_LED_RED_A, HIGH);       //赤信号に戻す
    digitalWrite(PIN_LED_GREEN_A, LOW);       //赤信号に戻す
    digitalWrite(PIN_LED_RED_B, HIGH);      //赤信号に戻す
    digitalWrite(PIN_LED_GREEN_B, LOW);       //赤信号に戻す
}


/* 100msに一度モーターの状態をアップデートする */
/* この関数の直前にスピード調整用VOLを読む */
void set_motor_duty(Control_t* control){
    switch(control->state){
        case STATE_TRAIN_ENTRY:
            control->pwm_duty -= SLOW_DOWN_STEP;
            if(control->pwm_duty < minimum_pwm_duty){
                control->pwm_duty = minimum_pwm_duty;
            }
            break;

        case STATE_TRAIN_WAIT :
            control->pwm_duty = maximum_pwm_duty;
            break;

        default : //case STATE_TRAIN_STOP :
            control->pwm_duty = 0;
            break;

    }
    if(control->last_pwm_duty != control->pwm_duty){            //違うときだけ更新
        analogWrite(control->pin_pwm_out ,control->pwm_duty);
        control->last_pwm_duty = control->pwm_duty;
    }
}

/* デバッグ用で閾値をシリアルに出す */
void debug_vol(){
    Serial.print("VT:");
    Serial.print(analogReadHiz(PIN_VOL_THRESHOLD));
    Serial.print(", SPA:");
    Serial.println(analogReadHiz(PIN_SENS_STOP_A));
}

/* 100msのタイマで呼び出されてモーターのデューティー比を変更する*/
void update_train_speed(){
    //debug_vol();
    static uint8_t onb_led_state = 0;
    digitalWrite(PIN_ONB_LED, onb_led_state);
    onb_led_state = !onb_led_state;

    maximum_pwm_duty = analogReadHiz(PIN_SENS_MAX_DUTY) >> 2;
    minimum_pwm_duty = analogReadHiz(PIN_SENS_MIN_DUTY) >> 2;
    set_motor_duty(&control_a);
    set_motor_duty(&control_b);
}

/* 自動運転のメイン処理（ステートマシン的なもの） */
/* A,Bで同じ処理が２回あるので、構造体のポインタをもらって処理する */
void proc_train_control( Control_t* control){
    switch(control->state){
        case STATE_TRAIN_WAIT:                  //進入待ち
            if(control->sens_slow){             //進入検出
                control->state = STATE_TRAIN_ENTRY;
                digitalWrite(control->pin_led_red, HIGH);       //黄色信号
                digitalWrite(control->pin_led_green, HIGH);
            }
            break;
        case STATE_TRAIN_ENTRY:                 //停止位置待ち
            if(control->sens_stop){             //停止位置検出
                control->state = STATE_TRAIN_STOP;
                digitalWrite(control->pin_led_red, LOW);       //緑信号
                digitalWrite(control->pin_led_green, HIGH);
                control->pwm_duty = 0;
                analogWrite(control->pin_pwm_out ,control->pwm_duty);   // タイマー割り込みを待つと行きすぎるので即停止
            }
            break;

        // case STATE_TRAIN_STOP: STATE_TRAIN_STOP → STATE_TRAIN_WAIT は train_go の reset_train_control で遷移する
        // このときVOLがセンターのとき約5秒待ち、その間に sens_slow と sens_stop はHになる

        default:
            break;
    }

}



void setup() {
    Serial.begin(9600);
    pinMode(PIN_DUO_SEL, INPUT_PULLUP);
    pinMode(PIN_BUTTON_INIT, INPUT_PULLUP);
    control_a.pin_led_red = PIN_LED_RED_A;
    control_b.pin_led_red = PIN_LED_RED_B;
    control_a.pin_led_green = PIN_LED_GREEN_A;
    control_b.pin_led_green = PIN_LED_GREEN_B;
    control_a.pin_pwm_out = PIN_MOTOR_A;
    control_b.pin_pwm_out = PIN_MOTOR_B;

    init_train_control(&control_a);
    init_train_control(&control_b);

    // 閾値を半固定抵抗で調整するため、列車を停止センサの上にセットし、セットされたら黄→緑信号にする。
    // いきなりスタートしないように起動完了ボタン押下待ち
    while(digitalRead(PIN_BUTTON_INIT) == HIGH){
            uint16_t refrector_threshold = analogReadHiz(PIN_VOL_THRESHOLD);
            check_sensor();
            digitalWrite(PIN_LED_RED_A, !control_a.sens_stop);
            digitalWrite(PIN_LED_RED_B, !control_b.sens_stop);
            delay(10);
    }
    while(digitalRead(PIN_BUTTON_INIT) == LOW){ //次は離されるの待ち
        delay(100);
    }

    MsTimer2::set(100, update_train_speed);
    MsTimer2::start();

    train_go();
}



void loop() {
    check_sensor();
    proc_train_control(&control_a);
    proc_train_control(&control_b);

    if (check_ready_go()){
        train_go();
    }
}
