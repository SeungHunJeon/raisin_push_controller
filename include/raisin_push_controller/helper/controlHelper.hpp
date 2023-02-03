#pragma once

Eigen::Vector2d ovalSampler(const double &speed, const double &ang, const double &forward, const double &backward, const double &sideways) {
  Eigen::Vector2d vel;
  vel << ((forward + backward) / 2.) * speed * cos(M_PI * ang) + ((forward - backward) / 2.),
      sideways * speed * sin(M_PI * ang);
  return vel;
}

Eigen::Vector2d eggSampler(const double &speed, const double &ang, const double &forward, const double &backward, const double &sideways) {
  Eigen::Vector2d vel;
  vel << speed * cos(M_PI * ang),
      sideways * speed * sin(M_PI * ang);
  if (vel[0] < 0)
    vel[0] *= backward;
  else
    vel[0] *= forward;
  return vel;
}

Eigen::VectorXd noisyGC(const Eigen::VectorXd &gc_init_,
                        std::normal_distribution<double> &normDist_, std::mt19937 &gen_) {
  Eigen::VectorXd gc(gc_init_.size());
  for (int i = 0; i < gc.size(); ++i) {
    if (i < 3) continue;
    gc[i] = gc_init_[i] + normDist_(gen_) * 0.2;
  }
  if (normDist_(gen_) < 0)
    gc[6] *= -1;

  gc.segment(3, 4) /= gc.segment(3, 4).norm();
  return gc;
}

Eigen::VectorXd noisyGV(const Eigen::VectorXd &gv_init_,
                        std::normal_distribution<double> &normDist_, std::mt19937 &gen_) {
  Eigen::VectorXd gv(gv_init_.size());
  for (int i = 0; i < gv.size(); ++i) {
    if (i == 0) { gv[i] = gv_init_[i] + normDist_(gen_) * 0.20; continue; } // x vel
    if (i == 1) { gv[i] = gv_init_[i] + normDist_(gen_) * 0.15; continue; } // y vel
    if (i == 2) { gv[i] = gv_init_[i] + normDist_(gen_) * 0.10; continue; } // z vel
    if (i <  6) { gv[i] = gv_init_[i] + normDist_(gen_) * 0.20; continue; } // rpy vel
    gv[i] = gv_init_[i] + normDist_(gen_) * 1.25; // joint speed
  }
  return gv;
}