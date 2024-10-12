#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include "../../WBC/ContactSpec.hpp"
#include "../WBCDataBuffer.h"
#include "memory"


class SingleContact : public ContactSpec<float> {
 public:
  SingleContact(std::shared_ptr<WBCDataBuffer> wbc_data, int pt);
  virtual ~SingleContact();

  void setMaxFz(float max_fz) { _max_Fz = max_fz; }

 protected:
  float _max_Fz;
  int _contact_pt;
  int _dim_U;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

    std::shared_ptr<WBCDataBuffer> wbc_data_;
};

#endif
