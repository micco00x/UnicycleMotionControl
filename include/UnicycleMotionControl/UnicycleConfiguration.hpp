#pragma once

namespace labrob {

class UnicycleConfiguration {
 public:
  UnicycleConfiguration();
  UnicycleConfiguration(double x, double y, double theta);

  double x() const;
  double& x();

  double y() const;
  double& y();

  double theta() const;
  double& theta();
  
 protected:
  double x_;
  double y_;
  double theta_;

}; // end class UnicycleConfiguration

} // end namespace labrob