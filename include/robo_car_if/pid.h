#ifndef PID_H_
#define PID_H_

namespace pid {

typedef struct {
  float kp;
  float ki;
  float kd;
  float min;
  float max;
  float dt;
  float tol;
} Params_T;

class PID {
 private:
  float kp_;
  float ki_;
  float kd_;
  float dt_;
  float tol_; /* Tolerance */
  float last_e_;
  float integral_;
 public:
  PID(Params_T* params);
  float Step(float error, float max, float min);
  void  Reset(void);
};

}  // namespace pid

#endif  // PID_H_