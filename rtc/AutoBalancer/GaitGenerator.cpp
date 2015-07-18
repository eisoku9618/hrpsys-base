/* -*- coding:utf-8-unix; mode:c++; -*- */

#include "GaitGenerator.h"
#include <numeric>
namespace rats
{
#ifndef rad2deg
#define rad2deg(rad) (rad * 180 / M_PI)
#endif
#ifndef deg2rad
#define deg2rad(deg) (deg * M_PI / 180)
#endif
  void cycloid_midpoint (hrp::Vector3& ret,
                         const double ratio, const hrp::Vector3& start,
                         const hrp::Vector3& goal, const double height,
                         const double default_top_ratio)
  {
    hrp::Vector3 u ( goal - start );
    hrp::Vector3 uz (0,0, ratio * u(2));
    u(2) = 0.0;
    double pth = 2 * M_PI * ratio, norm_u = u.norm();
    if ( !eps_eq(norm_u, 0.0,1e-3*0.01) )
      u =  u.normalized();
    /* check ratio vs 0.5 for default_top_ratio blending */
    hrp::Vector3 cycloid_point( ((0.5 > ratio) ? ( 2 * default_top_ratio * norm_u ) : ( 2 * (1 - default_top_ratio) * norm_u )) * ( pth - sin(pth) ) / (2 * M_PI) -
			   ((0.5 > ratio) ? 0.0 : (norm_u * (1 - 2 * default_top_ratio)) ), // local x
			   0, // local y
			   ( 0.5 * height * ( 1 - cos(pth) )) ); // local z
    hrp::Vector3 v(hrp::Vector3(0,0,1).cross(u));
    hrp::Matrix33 dvm;
    dvm << u(0), v(0), 0,
      u(1), v(1), 0,
      u(2), v(2), 1;
    ret = dvm * cycloid_point + start + uz;
  };

    std::vector<leg_type> get_support_leg_type_list_from_footstep_node_list(const std::vector<step_node>& fnl) {
        if (fnl.size() == 1) {
            std::vector<leg_type> tmp_spll = boost::assign::list_of(RLEG)(LLEG);
            /* std::vector<leg_type> tmp_spll; tmp_spll += RLEG, LLEG; */
            boost::remove_erase_if(tmp_spll, (boost::lambda::_1 == fnl.front().l_r));
            return tmp_spll;
        } else if (fnl.size() == 2) {
            std::vector<leg_type> tmp_spll = boost::assign::list_of(RLEG)(LLEG)(RARM)(LARM);
            boost::remove_erase_if(tmp_spll, (boost::lambda::_1 == fnl.at(0).l_r || boost::lambda::_1 == fnl.at(1).l_r));
            return tmp_spll;
        } else {
            std::cerr << "invalid !!!1" << std::endl;
            exit(1);
        }
    };

  /* member function implementation for refzmp_generator */
  void refzmp_generator::push_refzmp_from_footstep_list_for_dual (const std::vector<step_node>& fnl_kuro,
                                                                  const std::vector<coordinates>& _support_leg_coords_list_kuro,
                                                                  const std::vector<coordinates>& _swing_leg_coords_list_kuro)
  {
    hrp::Vector3 rzmp;
    std::vector<hrp::Vector3> dzl;
    hrp::Vector3 ret_zmp;
    hrp::Vector3 tmp_zero = hrp::Vector3::Zero();
    std::vector<leg_type> spll = get_support_leg_type_list_from_footstep_node_list(fnl_kuro);
    for (size_t i = 0, len = _support_leg_coords_list_kuro.size(); i < len; i++) {
      dzl.push_back(_support_leg_coords_list_kuro.at(i).rot * default_zmp_offsets[spll.at(i)] + _support_leg_coords_list_kuro.at(i).pos);
    }
    for (size_t i = 0, len = _swing_leg_coords_list_kuro.size(); i < len; i++) {
      dzl.push_back(_swing_leg_coords_list_kuro.at(i).rot * default_zmp_offsets[fnl_kuro.at(i).l_r] + _swing_leg_coords_list_kuro.at(i).pos);
    }
    rzmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / dzl.size();
    refzmp_cur_list.push_back( rzmp );
    std::vector<hrp::Vector3> foot_x_axis_list;
    for (size_t i = 0; i < _swing_leg_coords_list_kuro.size(); i++) {
        foot_x_axis_list.push_back( hrp::Vector3(_swing_leg_coords_list_kuro.at(i).rot * hrp::Vector3::UnitX()) );
    }
    foot_x_axis_list_list_kuro.push_back(foot_x_axis_list);
    std::vector<leg_type> swing_leg_list;
    for (size_t i = 0; i < fnl_kuro.size(); i++) {
        swing_leg_list.push_back(fnl_kuro.at(i).l_r);
    }
    swing_leg_list_list_kuro.push_back( swing_leg_list );
    step_count_list.push_back(static_cast<size_t>(fnl_kuro.front().step_time/dt));
    //std::cerr << "double " << (fnl[fs_index].l_r==RLEG?LLEG:RLEG) << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
  };

  void refzmp_generator::push_refzmp_from_footstep_list_for_single (const std::vector<step_node>& fnl_kuro, const std::vector<coordinates>& _support_leg_coords_list_kuro)
  {
    // support leg = prev fnl l_r
    // swing leg = fnl l_r
      hrp::Vector3 rzmp, tmp_zero=hrp::Vector3::Zero();
    std::vector<hrp::Vector3> dzl;

    std::vector<leg_type> spll = get_support_leg_type_list_from_footstep_node_list(fnl_kuro);
    for (size_t i = 0, len = _support_leg_coords_list_kuro.size(); i < len; i++) {
      dzl.push_back(_support_leg_coords_list_kuro.at(i).rot * default_zmp_offsets[spll.at(i)] + _support_leg_coords_list_kuro.at(i).pos);
    }
    rzmp = std::accumulate(dzl.begin(), dzl.end(), tmp_zero) / dzl.size();
    refzmp_cur_list.push_back( rzmp );
    std::vector<hrp::Vector3> foot_x_axis_list;
    for (size_t i = 0; i < _support_leg_coords_list_kuro.size(); i++) {
        foot_x_axis_list.push_back( hrp::Vector3(_support_leg_coords_list_kuro.at(i).rot * hrp::Vector3::UnitX()) );
    }
    foot_x_axis_list_list_kuro.push_back(foot_x_axis_list);
    std::vector<leg_type> swing_leg_list;
    for (size_t i = 0; i< fnl_kuro.size(); i++) {
        swing_leg_list.push_back(fnl_kuro.at(i).l_r);
    }
    swing_leg_list_list_kuro.push_back( swing_leg_list );
    step_count_list.push_back(static_cast<size_t>(fnl_kuro.front().step_time/dt));
    //std::cerr << "single " << fnl[fs_index-1].l_r << " [" << refzmp_cur_list.back()(0) << " " << refzmp_cur_list.back()(1) << " " << refzmp_cur_list.back()(2) << "]" << std::endl;
  };

    /* only biped */
  void refzmp_generator::calc_current_refzmp (hrp::Vector3& ret, std::vector<hrp::Vector3>& swing_foot_zmp_offset_list_kuro, const double default_double_support_ratio, const double default_double_support_static_ratio) const
  {
    size_t cnt = one_step_count - refzmp_count; // current counter (0 -> one_step_count)
    size_t double_support_count_half = (0.5 * default_double_support_ratio) * one_step_count;
    size_t double_support_static_count_half = (0.5 * default_double_support_static_ratio) * one_step_count;
    for (size_t i = 0; i < swing_leg_list_list_kuro[refzmp_index].size(); i++) {
        swing_foot_zmp_offset_list_kuro.push_back(default_zmp_offsets[swing_leg_list_list_kuro[refzmp_index].at(i)]);
    }
    double zmp_diff = 0.0; // difference between total swing_foot_zmp_offset and default_zmp_offset
    //if (cnt==0) std::cerr << "z " << refzmp_index << " " << refzmp_cur_list.size() << " " << fs_index << " " << (refzmp_index == refzmp_cur_list.size()-2) << " " << is_final_double_support_set << std::endl;

    // Calculate swing foot zmp offset for toe heel zmp transition
    if (use_toe_heel_transition &&
        !(is_start_double_support_phase() || is_end_double_support_phase())) { // Do not use toe heel zmp transition during start and end double support period because there is no swing foot
        if (thp_ptr->is_between_phases(cnt, SOLE0)) {
            double ratio = thp_ptr->calc_phase_ratio(cnt+1, SOLE0);
            swing_foot_zmp_offset_list_kuro.front()(0) = (1-ratio)*swing_foot_zmp_offset_list_kuro.front()(0) + ratio*toe_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, HEEL2SOLE, SOLE2)) {
            double ratio = thp_ptr->calc_phase_ratio(cnt, HEEL2SOLE, SOLE2);
            swing_foot_zmp_offset_list_kuro.front()(0) = ratio*swing_foot_zmp_offset_list_kuro.front()(0) + (1-ratio)*heel_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, SOLE0, SOLE2TOE)) {
            swing_foot_zmp_offset_list_kuro.front()(0) = toe_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, SOLE2HEEL, HEEL2SOLE)) {
            swing_foot_zmp_offset_list_kuro.front()(0) = heel_zmp_offset_x;
        } else if (thp_ptr->is_between_phases(cnt, SOLE2TOE, SOLE2HEEL)) {
            double ratio = thp_ptr->calc_phase_ratio(cnt, SOLE2TOE, SOLE2HEEL);
            swing_foot_zmp_offset_list_kuro.front()(0) = ratio * heel_zmp_offset_x + (1-ratio) * toe_zmp_offset_x;
        }
        zmp_diff = swing_foot_zmp_offset_list_kuro.front()(0)-default_zmp_offsets[swing_leg_list_list_kuro[refzmp_index].front()](0);
        if ((is_second_phase() && ( cnt < double_support_count_half )) ||
            (is_second_last_phase() && ( cnt > one_step_count - double_support_count_half ))) {
            // "* 0.5" is for double supprot period
            zmp_diff *= 0.5;
        }
    }

    // Calculate total reference ZMP
    if (is_start_double_support_phase() || is_end_double_support_phase()) {
      ret = refzmp_cur_list[refzmp_index];
    } else if ( cnt < double_support_static_count_half ) { // Start double support static period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axis_list_list_kuro[refzmp_index-1].front();
      double ratio = (is_second_phase()?1.0:0.5); 
      ret = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_static_count_half ) { // End double support static period
        hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axis_list_list_kuro[refzmp_index+1].front();
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
      double ratio = (is_second_last_phase()?1.0:0.5);
      ret = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else if ( cnt < double_support_count_half ) { // Start double support period
      hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index];
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index-1] + zmp_diff * foot_x_axis_list_list_kuro[refzmp_index-1].front();
      double ratio = ((is_second_phase()?1.0:0.5) / (double_support_count_half-double_support_static_count_half)) * (double_support_count_half-cnt);
      ret = (1 - ratio) * current_support_zmp + ratio * prev_support_zmp;
    } else if ( cnt > one_step_count - double_support_count_half ) { // End double support period
        hrp::Vector3 current_support_zmp = refzmp_cur_list[refzmp_index+1] + zmp_diff * foot_x_axis_list_list_kuro[refzmp_index+1].front();
      hrp::Vector3 prev_support_zmp = refzmp_cur_list[refzmp_index];
      double ratio = ((is_second_last_phase()?1.0:0.5) / (double_support_count_half-double_support_static_count_half)) * (cnt - 1 - (one_step_count - double_support_count_half));
      ret = (1 - ratio) * prev_support_zmp + ratio * current_support_zmp;
    } else {
      ret = refzmp_cur_list[refzmp_index];
    }
  };

  void refzmp_generator::update_refzmp (const std::vector< std::vector<step_node> >& fnll)
  {
    if ( 1 <= refzmp_count ) {
      refzmp_count--;
    } else {
      refzmp_index++;
      refzmp_count = one_step_count = step_count_list[refzmp_index];
      //std::cerr << "fs " << fs_index << "/" << fnl.size() << " rf " << refzmp_index << "/" << refzmp_cur_list.size() << " flg " << std::endl;
    }
  };

  /* member function implementation for leg_coords_generator */
  void leg_coords_generator::calc_current_swing_leg_coords_list_kuro (std::vector<coordinates>& ret_list, const double step_height, const double _current_toe_angle, const double _current_heel_angle)
  {
    for (std::vector<coordinates>::iterator it1 = swing_leg_src_coords_list_kuro.begin(), it2 = swing_leg_dst_coords_list_kuro.begin();
         it1 != swing_leg_src_coords_list_kuro.end() && it2 != swing_leg_dst_coords_list_kuro.end();
         it1++, it2++) {
      coordinates ret;
      switch (default_orbit_type) {
      case SHUFFLING:
        mid_coords(ret, swing_rot_ratio, *it1, *it2);
        break;
      case CYCLOID:
        cycloid_midcoords(ret, *it1, *it2, step_height);
        break;
      case RECTANGLE:
        rectangle_midcoords(ret, *it1, *it2, step_height);
        break;
      case STAIR:
        stair_midcoords(ret, *it1, *it2, step_height);
        break;
      case CYCLOIDDELAY:
        cycloid_delay_midcoords(ret, *it1, *it2, step_height);
        break;
      case CYCLOIDDELAYKICK:
        cycloid_delay_kick_midcoords(ret, *it1, *it2, step_height);
        break;
      default: break;
      }
      if (std::fabs(step_height) > 1e-3*10) {
        modif_foot_coords_for_toe_heel_phase(ret, _current_toe_angle, _current_heel_angle);
      }
      ret_list.push_back(ret);
    }
  };

  void leg_coords_generator::calc_ratio_from_double_support_ratio (const double default_double_support_ratio)
  {
    int swing_len = (1.0 - default_double_support_ratio) * one_step_count;
    int support_len = one_step_count - swing_len;
    int current_swing_len = lcg_count - support_len/2;
    double tmp_current_swing_time;
    int current_swing_count = (one_step_count - lcg_count); // 0->one_step_count
    if ( current_swing_count < support_len/2 ) { // First double support period
      swing_ratio = swing_rot_ratio = 0.0;
      tmp_current_swing_time = current_swing_len * dt - swing_len * dt;
    } else if ( current_swing_count >= support_len/2+swing_len ) { // Last double support period
      swing_ratio = swing_rot_ratio = 1.0;
      tmp_current_swing_time = current_swing_len * dt + (default_double_support_ratio * one_step_count + next_one_step_count) * dt;
    } else {
      if (current_swing_count == support_len/2) {
          double tmp = 0.0;
          swing_foot_rot_ratio_interpolator->clear();
          swing_foot_rot_ratio_interpolator->set(&tmp);
          tmp = 1.0;
          // int reduced_swing_len = 0.95*swing_len; // For margin from early landing
          // swing_foot_rot_ratio_interpolator->go(&tmp, dt * reduced_swing_len);
          //swing_foot_rot_ratio_interpolator->go(&tmp, dt * swing_len);
          swing_foot_rot_ratio_interpolator->setGoal(&tmp, dt * swing_len);
          swing_foot_rot_ratio_interpolator->sync();
      }
      if (!swing_foot_rot_ratio_interpolator->isEmpty()) {
          swing_foot_rot_ratio_interpolator->get(&swing_rot_ratio, true);
      } else {
          swing_foot_rot_ratio_interpolator->get(&swing_rot_ratio, false);
      }
      tmp_current_swing_time = current_swing_len * dt;
      swing_ratio = static_cast<double>(current_swing_count-support_len/2)/swing_len;
      //std::cerr << "gp " << swing_ratio << " " << swing_rot_ratio << std::endl;
    }
    /* only biped */
    current_swing_time[support_leg_list_kuro.front()] = (lcg_count + 0.5 * default_double_support_ratio * next_one_step_count) * dt;
    current_swing_time[support_leg_list_kuro.front()==RLEG ? LLEG : RLEG] = tmp_current_swing_time;
    //std::cerr << "sl " << support_leg << " " << current_swing_time[support_leg==RLEG?0:1] << " " << current_swing_time[support_leg==RLEG?1:0] << " " << tmp_current_swing_time << " " << lcg_count << std::endl;
  };

  double leg_coords_generator::calc_interpolated_toe_heel_angle (const toe_heel_phase start_phase, const toe_heel_phase goal_phase, const double start, const double goal)
  {
      double tmp_ip_ratio;
      size_t current_count = one_step_count - lcg_count;
      if (thp_ptr->is_phase_starting(current_count, start_phase)) {
          toe_heel_interpolator->clear();
          toe_heel_interpolator->set(&start);
          //toe_heel_interpolator->go(&goal, thp_ptr->calc_phase_period(start_phase, goal_phase, dt));
          toe_heel_interpolator->setGoal(&goal, thp_ptr->calc_phase_period(start_phase, goal_phase, dt));
          toe_heel_interpolator->sync();
      }
      if (!toe_heel_interpolator->isEmpty()) {
          toe_heel_interpolator->get(&tmp_ip_ratio, true);
      } else {
          toe_heel_interpolator->get(&tmp_ip_ratio, false);
      }
      return tmp_ip_ratio;
  };

  void leg_coords_generator::modif_foot_coords_for_toe_heel_phase (coordinates& org_coords, const double _current_toe_angle, const double _current_heel_angle)
  {
      coordinates new_coords;
      size_t current_count = one_step_count - lcg_count;
      double dif_angle = 0.0;
      hrp::Vector3 ee_local_pivot_pos(hrp::Vector3(0,0,0));
      if ( thp_ptr->is_between_phases(current_count, SOLE0, SOLE2TOE) ) {
          dif_angle = calc_interpolated_toe_heel_angle(SOLE0, SOLE2TOE, 0.0, _current_toe_angle);
          ee_local_pivot_pos(0) = toe_pos_offset_x;
      } else if ( thp_ptr->is_between_phases(current_count, SOLE2HEEL, HEEL2SOLE) ) {
          dif_angle = calc_interpolated_toe_heel_angle(SOLE2HEEL, HEEL2SOLE, -1 * _current_heel_angle, 0.0);
          ee_local_pivot_pos(0) = heel_pos_offset_x;
      } else if ( thp_ptr->is_between_phases(current_count, SOLE2TOE, SOLE2HEEL) ) {
          // If SOLE1 phase does not exist, interpolate toe => heel smoothly, without 0 velocity phase.
          if ( thp_ptr->is_no_SOLE1_phase() ) {
              dif_angle = calc_interpolated_toe_heel_angle(SOLE2TOE, SOLE2HEEL, _current_toe_angle, -1 * _current_heel_angle);
              double tmpd = (-1*_current_heel_angle-_current_toe_angle);
              if (std::fabs(tmpd) > 1e-5) {
                  ee_local_pivot_pos(0) = (heel_pos_offset_x - toe_pos_offset_x) * (dif_angle - _current_toe_angle) / tmpd + toe_pos_offset_x;
              }
          } else {
              if ( thp_ptr->is_between_phases(current_count, SOLE2TOE, TOE2SOLE) ) {
                  dif_angle = calc_interpolated_toe_heel_angle(SOLE2TOE, TOE2SOLE, _current_toe_angle, 0.0);
                  ee_local_pivot_pos(0) = toe_pos_offset_x;
              } else if ( thp_ptr->is_between_phases(current_count, SOLE1, SOLE2HEEL) ) {
                  dif_angle = calc_interpolated_toe_heel_angle(SOLE1, SOLE2HEEL, 0.0, -1 * _current_heel_angle);
                  ee_local_pivot_pos(0) = heel_pos_offset_x;
              }
          }
      }
      foot_dif_rot_angle = (dif_angle > 0.0 ? deg2rad(dif_angle) : 0.0);
      if (use_toe_joint && dif_angle > 0.0) dif_angle = 0.0;
      Eigen::AngleAxis<double> tmpr(deg2rad(dif_angle), hrp::Vector3::UnitY());
      rotm3times(new_coords.rot, org_coords.rot, tmpr.toRotationMatrix());
      new_coords.pos = org_coords.pos + org_coords.rot * ee_local_pivot_pos - new_coords.rot * ee_local_pivot_pos;
      org_coords = new_coords;
  };

  void leg_coords_generator::cycloid_midcoords (coordinates& ret, const coordinates& start,
                                                                const coordinates& goal, const double height) const
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    cycloid_midpoint (ret.pos, swing_ratio, start.pos, goal.pos, height, default_top_ratio);
  };

  void leg_coords_generator::rectangle_midcoords (coordinates& ret, const coordinates& start,
                                                                  const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    rdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::stair_midcoords (coordinates& ret, const coordinates& start,
                                                              const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    sdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cycloid_delay_midcoords (coordinates& ret, const coordinates& start,
                                                                      const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    cdtg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::cycloid_delay_kick_midcoords (coordinates& ret, const coordinates& start,
                                                                      const coordinates& goal, const double height)
  {
    mid_coords(ret, swing_rot_ratio, start, goal);
    cdktg.set_start_rot(hrp::Matrix33(start.rot));
    cdktg.get_trajectory_point(ret.pos, hrp::Vector3(start.pos), hrp::Vector3(goal.pos), height);
  };

  void leg_coords_generator::update_leg_coords (const std::vector< std::vector<step_node> >& fnll_kuro, const double default_double_support_ratio)
  {
    if (!foot_ratio_interpolator->isEmpty()) {
        foot_ratio_interpolator->get(&foot_midcoords_ratio, true);
    }

    // Get current swing coords, support coords, and support leg parameters
    size_t current_footstep_index = (footstep_index < fnll_kuro.size() - 1 ? footstep_index : fnll_kuro.size()-1);
    /* koko */
    swing_leg_dst_coords_list_kuro.clear();
    for (size_t i = 0; i < fnll_kuro[current_footstep_index].size(); i++) {
        swing_leg_dst_coords_list_kuro.push_back(fnll_kuro[current_footstep_index].at(i).worldcoords);
    }
    support_leg_list_kuro = get_support_leg_type_list_from_footstep_node_list(fnll_kuro[current_footstep_index]);
    if (footstep_index != 0) { // If not initial step, support_leg_coords is previous swing_leg_dst_coords
        support_leg_coords_list_kuro = support_leg_coords_list_list_kuro[current_footstep_index];
    }
    /* only biped ? */
    if (current_footstep_index > 0) {
      if (fnll_kuro[current_footstep_index].front().l_r == fnll_kuro[current_footstep_index-1].front().l_r) {
            swing_leg_src_coords_list_kuro = swing_leg_dst_coords_list_list_kuro[current_footstep_index-1];
        } else {
            swing_leg_src_coords_list_kuro = support_leg_coords_list_list_kuro[current_footstep_index-1];
        }
    }

    calc_ratio_from_double_support_ratio(default_double_support_ratio);
    swing_leg_coords_list_kuro.clear();
    calc_current_swing_leg_coords_list_kuro(swing_leg_coords_list_kuro, current_step_height, current_toe_angle, current_heel_angle);
    if ( 1 <= lcg_count ) {
      lcg_count--;
    } else {
      //std::cerr << "gp " << footstep_index << std::endl;
      if (footstep_index < fnll_kuro.size() - 1) {
        footstep_index++;
      }
      if (footstep_index < fnll_kuro.size() - 1) {
        current_step_height = fnll_kuro[footstep_index].front().step_height;
        current_toe_angle = fnll_kuro[footstep_index].front().toe_angle;
        current_heel_angle = fnll_kuro[footstep_index].front().heel_angle;
      } else {
        current_step_height = current_toe_angle = current_heel_angle = 0.0;
      }
      if (footstep_index < fnll_kuro.size()) {
        one_step_count = static_cast<size_t>(fnll_kuro[footstep_index].front().step_time/dt);
      }
      if (footstep_index + 1 < fnll_kuro.size()) {
        next_one_step_count = static_cast<size_t>(fnll_kuro[footstep_index+1].front().step_time/dt);
      }
      lcg_count = one_step_count;
      rdtg.reset(one_step_count, default_double_support_ratio);
      sdtg.reset(one_step_count, default_double_support_ratio);
      cdtg.reset(one_step_count, default_double_support_ratio);
      cdktg.reset(one_step_count, default_double_support_ratio);      
      reset_foot_ratio_interpolator();
    }
  };

  /* member function implementation for gait_generator */
  void gait_generator::initialize_gait_parameter (const hrp::Vector3& cog,
                                                  const std::vector<coordinates>& initial_support_leg_coords_list_kuro,
                                                  const std::vector<coordinates>& initial_swing_leg_dst_coords_list_kuro,
                                                  const double delay)
  {
    /* clear all gait_parameter */
    size_t one_step_len = footstep_node_list_list_kuro[0][0].step_time / dt;
    finalize_count = 0;
    for (size_t i = 0; i < footstep_node_list_list_kuro[0].size(); i++) {
      footstep_node_list_list_kuro[0][i].worldcoords = initial_swing_leg_dst_coords_list_kuro[i];
    }
    rg.reset(one_step_len);
    rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list_list_kuro.front(), initial_support_leg_coords_list_kuro, initial_swing_leg_dst_coords_list_kuro);
    if ( preview_controller_ptr != NULL ) {
      delete preview_controller_ptr;
      preview_controller_ptr = NULL;
    }
    //preview_controller_ptr = new preview_dynamics_filter<preview_control>(dt, cog(2) - refzmp_cur_list[0](2), refzmp_cur_list[0]);
    preview_controller_ptr = new preview_dynamics_filter<extended_preview_control>(dt, cog(2) - rg.get_refzmp_cur()(2), rg.get_refzmp_cur(), gravitational_acceleration);
    lcg.reset(one_step_len, footstep_node_list_list_kuro[1][0].step_time/dt, initial_swing_leg_dst_coords_list_kuro, initial_swing_leg_dst_coords_list_kuro, initial_support_leg_coords_list_kuro, default_double_support_ratio);
    /* make another */
    lcg.set_swing_support_list_list_kuro(footstep_node_list_list_kuro);
    for (size_t i = 1; i < footstep_node_list_list_kuro.size()-1; i++) {
        rg.push_refzmp_from_footstep_list_for_single(footstep_node_list_list_kuro[i], lcg.get_support_leg_coords_list_kuro_idx(i));
    }
    rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list_list_kuro[footstep_node_list_list_kuro.size()-1],
                                               lcg.get_swing_leg_dst_coords_list_kuro_idx(footstep_node_list_list_kuro.size()-1),
                                               lcg.get_support_leg_coords_list_kuro_idx(footstep_node_list_list_kuro.size()-1));
    emergency_flg = IDLING;
  };

  bool gait_generator::proc_one_tick ()
  {
    hrp::Vector3 rzmp;
    std::vector<hrp::Vector3> sfzo_list_kuro;
    bool refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzo_list_kuro, default_double_support_ratio, default_double_support_static_ratio);
    if (!refzmp_exist_p) {
      finalize_count++;
      rzmp = prev_que_rzmp;
      sfzo_list_kuro = prev_que_sfzo_list_kuro;
    } else {
      prev_que_rzmp = rzmp;
      prev_que_sfzo_list_kuro = sfzo_list_kuro;
    }
    /* only biped */
    bool solved = preview_controller_ptr->update(refzmp, cog, swing_foot_zmp_offset_list_kuro.front(), rzmp, sfzo_list_kuro.front(), (refzmp_exist_p || finalize_count < preview_controller_ptr->get_delay()-default_step_time/dt));
    /* update refzmp */
    if ( lcg.get_footstep_index() > 0 &&
         lcg.get_lcg_count() == static_cast<size_t>(footstep_node_list_list_kuro[lcg.get_footstep_index()][0].step_time/dt * 0.5) - 1 ) { // Almost middle of step time
      if (velocity_mode_flg != VEL_IDLING) {
        std::vector< std::vector<coordinates> > cv;
        calc_next_coords_velocity_mode(cv, lcg.get_footstep_index() + 1);
        if (velocity_mode_flg == VEL_ENDING) velocity_mode_flg = VEL_IDLING;
        std::vector<leg_type> cur_leg;
        for (size_t i = 0; i < footstep_node_list_list_kuro[lcg.get_footstep_index()].size(); i++) {
            cur_leg.push_back(footstep_node_list_list_kuro[lcg.get_footstep_index()].at(i).l_r);
        }
        /* only biped */
        overwrite_footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(cur_leg.front()==RLEG?LLEG:RLEG, cv[0][0], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle())));
        overwrite_footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(cur_leg.front(), cv[1][0], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle())));
        overwrite_footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(cur_leg.front()==RLEG?LLEG:RLEG, cv[2][0], lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle())));
        overwrite_refzmp_queue(overwrite_footstep_node_list_list_kuro);
        overwrite_footstep_node_list_list_kuro.clear();
      } else if (!overwrite_footstep_node_list_list_kuro.empty() && (lcg.get_footstep_index() < footstep_node_list_list_kuro.size()-1) ) { // If overwrite_footstep_node_list is specified and current footstep is not last footstep.
        overwrite_refzmp_queue(overwrite_footstep_node_list_list_kuro);
        overwrite_footstep_node_list_list_kuro.clear();
      } else if (emergency_flg == EMERGENCY_STOP) {
        leg_type cur_leg = footstep_node_list_list_kuro[lcg.get_footstep_index()].front().l_r;
        overwrite_footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(cur_leg==RLEG?LLEG:RLEG, footstep_node_list_list_kuro[lcg.get_footstep_index()-1].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(cur_leg, footstep_node_list_list_kuro[lcg.get_footstep_index()].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(cur_leg==RLEG?LLEG:RLEG, footstep_node_list_list_kuro[lcg.get_footstep_index()-1].front().worldcoords, 0, default_step_time, 0, 0)));
        overwrite_refzmp_queue(overwrite_footstep_node_list_list_kuro);
        overwrite_footstep_node_list_list_kuro.clear();
        emergency_flg = STOPPING;
      }
    }
    rg.update_refzmp(footstep_node_list_list_kuro);
    // { // debug
    //   double cart_zmp[3];
    //   preview_controller_ptr->get_cart_zmp(cart_zmp);
    //   std::cerr << "(list " << std::endl;
    //   std::cerr << ":cog "; print_vector(std::cerr, cog);
    //   std::cerr << ":refzmp "; print_vector(std::cerr, refzmp);
    //   std::cerr << ":cart-zmp "; print_vector(std::cerr, cart_zmp, 3);
    //   std::cerr << ")" << std::endl;
    // }

    /* update swing_leg_coords, support_leg_coords */
    if ( solved ) {
      lcg.update_leg_coords(footstep_node_list_list_kuro, default_double_support_ratio);
    } else if (finalize_count>0) {
      lcg.clear_interpolators();
    }
    return solved;
  };

  /* generate vector of step_node from :go-pos params
   *  x, y and theta are simply divided by using stride params
   *  unit system -> x [mm], y [mm], theta [deg]
   */
  void gait_generator::go_pos_param_2_footstep_list_list_kuro (const double goal_x, const double goal_y, const double goal_theta, /* [mm] [mm] [deg] */
                                       const coordinates& initial_support_coords, const coordinates& initial_swing_src_coords,
                                       const leg_type initial_support_leg)
  {
    coordinates foot_midcoords; /* foot_midcoords is modified during loop */
    mid_coords(foot_midcoords, 0.5, initial_support_coords, initial_swing_src_coords);
    coordinates goal_foot_midcoords(foot_midcoords);
    goal_foot_midcoords.pos += goal_foot_midcoords.rot * hrp::Vector3(goal_x, goal_y, 0.0);
    goal_foot_midcoords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    std::cerr << "current foot midcoords" << std::endl;
    std::cerr << "  pos =" << std::endl;
    std::cerr << foot_midcoords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rot =" << std::endl;
    std::cerr << foot_midcoords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    std::cerr << "goal foot midcoords" << std::endl;
    std::cerr << "  pos =" << std::endl;
    std::cerr << goal_foot_midcoords.pos.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << std::endl;
    std::cerr << "  rot =" << std::endl;
    std::cerr << goal_foot_midcoords.rot.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;

    /* initialize */
    clear_footstep_node_list_list_kuro();
    // For initial double support period
    footstep_node_list_list_kuro.push_back(boost::assign::list_of(step_node(initial_support_leg, initial_support_coords, 0, default_step_time, 0, 0)));

    /* footstep generation loop */
    hrp::Vector3 dp, dr;
    foot_midcoords.difference(dp, dr, goal_foot_midcoords);
    dp = foot_midcoords.rot.transpose() * dp;
    dr = foot_midcoords.rot.transpose() * dr;
    while ( !(eps_eq(dp.norm(), 0.0, 1e-3*0.1) && eps_eq(dr.norm(), 0.0, deg2rad(0.5))) ) {
      set_velocity_param(dp(0)/default_step_time, dp(1)/default_step_time, rad2deg(dr(2))/default_step_time);
      append_footstep_list_velocity_mode();
      foot_midcoords = footstep_node_list_list_kuro.back().front().worldcoords;
      foot_midcoords.pos += foot_midcoords.rot * hrp::Vector3(footstep_param.leg_default_translate_pos[footstep_node_list_list_kuro.back().front().l_r] * -1.0);
      foot_midcoords.difference(dp, dr, goal_foot_midcoords);
      dp = foot_midcoords.rot.transpose() * dp;
      dr = foot_midcoords.rot.transpose() * dr;
    }
    for (size_t i = 0; i < optional_go_pos_finalize_footstep_num; i++) {
      append_go_pos_step_node(foot_midcoords, (footstep_node_list_list_kuro.back().front().l_r == RLEG ? LLEG : RLEG));
    }

    /* finalize */
    //   Align last foot
    append_go_pos_step_node(foot_midcoords, (footstep_node_list_list_kuro.back().front().l_r == RLEG ? LLEG : RLEG));
    //   Check align
    coordinates final_step_coords1 = footstep_node_list_list_kuro[footstep_node_list_list_kuro.size()-2].front().worldcoords; // Final coords in footstep_node_list
    coordinates final_step_coords2 = foot_midcoords; // Final coords calculated from foot_midcoords + translate pos
    final_step_coords2.pos += final_step_coords2.rot * hrp::Vector3(footstep_param.leg_default_translate_pos[footstep_node_list_list_kuro[footstep_node_list_list_kuro.size()-2].front().l_r]);
    final_step_coords1.difference(dp, dr, final_step_coords2);
    if ( !(eps_eq(dp.norm(), 0.0, 1e-3*0.1) && eps_eq(dr.norm(), 0.0, deg2rad(0.5))) ) { // If final_step_coords1 != final_step_coords2, add steps to match final_step_coords1 and final_step_coords2
      append_go_pos_step_node(foot_midcoords, (footstep_node_list_list_kuro.back().front().l_r == RLEG ? LLEG : RLEG));
    }
    //   For Last double support period
    append_finalize_footstep();
    print_footstep_list_list_kuro();
  };

  void gait_generator::go_single_step_param_2_footstep_list_list_kuro (const double goal_x, const double goal_y, const double goal_z, const double goal_theta,
                                                             const std::string& tmp_swing_leg,
                                                             const coordinates& _support_leg_coords)
  {
    leg_type _swing_leg = (tmp_swing_leg == "rleg") ? RLEG : LLEG;
    step_node sn0((_swing_leg == RLEG) ? LLEG : RLEG, _support_leg_coords, lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle());
    footstep_node_list_list_kuro.push_back(boost::assign::list_of(sn0));
    step_node sn1(_swing_leg, _support_leg_coords, lcg.get_default_step_height(), default_step_time, lcg.get_toe_angle(), lcg.get_heel_angle());
    hrp::Vector3 trs(2.0 * footstep_param.leg_default_translate_pos[_swing_leg] + hrp::Vector3(goal_x, goal_y, goal_z));
    sn1.worldcoords.pos += sn1.worldcoords.rot * trs;
    sn1.worldcoords.rotate(deg2rad(goal_theta), hrp::Vector3(0,0,1));
    footstep_node_list_list_kuro.push_back(boost::assign::list_of(sn1));
    footstep_node_list_list_kuro.push_back(boost::assign::list_of(sn0));
  };

  void gait_generator::initialize_velocity_mode (const coordinates& _foot_midcoords,
						 const double vel_x, const double vel_y, const double vel_theta)
  {
    velocity_mode_flg = VEL_DOING;
    /* initialize */
    leg_type current_leg = (vel_y > 0.0) ? RLEG : LLEG;
    clear_footstep_node_list_list_kuro();
    set_velocity_param (vel_x, vel_y, vel_theta);
    append_go_pos_step_node(_foot_midcoords, current_leg);
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
    append_footstep_list_velocity_mode();
  };

  void gait_generator::finalize_velocity_mode ()
  {
    if (velocity_mode_flg == VEL_DOING) velocity_mode_flg = VEL_ENDING;
  };

  void gait_generator::calc_foot_midcoords_trans_vector_velocity_mode (std::vector<coordinates>& foot_midcoords_list_kuro, std::vector<hrp::Vector3>& trans_list_kuro, std::vector<double>& dth_list_kuro, const std::vector<step_node>& snl_kuro)
  {
    for (size_t i = 0; i < snl_kuro.size(); i++) {
      step_node sn = snl_kuro.at(i);
      coordinates foot_midcoords = sn.worldcoords;
      hrp::Vector3 tmpv(footstep_param.leg_default_translate_pos[sn.l_r] * -1.0);
      foot_midcoords.pos += foot_midcoords.rot * tmpv;
      foot_midcoords_list_kuro.push_back(foot_midcoords);
      double dx = vel_param.velocity_x + offset_vel_param.velocity_x, dy = vel_param.velocity_y + offset_vel_param.velocity_y;
      double dth = vel_param.velocity_theta + offset_vel_param.velocity_theta;
      /* velocity limitation by stride parameters <- this should be based on footstep candidates */
      if (footstep_param.stride_fwd_x / default_step_time < dx)
        dx = footstep_param.stride_fwd_x / default_step_time;
      if (-1*footstep_param.stride_bwd_x / default_step_time > dx)
        dx = -1*footstep_param.stride_bwd_x / default_step_time;
      if (footstep_param.stride_y / default_step_time < fabs(dy))
        dy = footstep_param.stride_y * ((dy > 0.0) ? 1.0 : -1.0) / default_step_time;
      if (footstep_param.stride_theta / default_step_time < fabs(dth))
        dth = footstep_param.stride_theta * ((dth > 0.0) ? 1.0 : -1.0) / default_step_time;
      /* inside step limitation */
      if (use_inside_step_limitation) {
        if (vel_param.velocity_y > 0) {
          if (sn.l_r == LLEG || sn.l_r == LARM) dy *= 0.5;
        } else {
          if (sn.l_r == RLEG || sn.l_r == RARM) dy *= 0.5;
        }
        if (vel_param.velocity_theta > 0) {
          if (sn.l_r == LLEG || sn.l_r == LARM) dth *= 0.5;
        } else {
          if (sn.l_r == RLEG || sn.l_r == RARM) dth *= 0.5;
        }
      }
      trans_list_kuro.push_back(hrp::Vector3(dx * default_step_time, dy * default_step_time, 0));
      dth_list_kuro.push_back(deg2rad(dth * default_step_time));
    }
  };

  void gait_generator::append_footstep_list_velocity_mode ()
  {
    std::vector<coordinates> foot_midcoords_list_kuro;
    std::vector<hrp::Vector3> trans_list_kuro;
    std::vector<double> dth_list_kuro;
    calc_foot_midcoords_trans_vector_velocity_mode(foot_midcoords_list_kuro, trans_list_kuro, dth_list_kuro, footstep_node_list_list_kuro.back());

    for (size_t i = 0; i < foot_midcoords_list_kuro.size(); i++) {
      foot_midcoords_list_kuro[i].pos += foot_midcoords_list_kuro[i].rot * trans_list_kuro[i];
      foot_midcoords_list_kuro[i].rotate(dth_list_kuro[i], hrp::Vector3(0,0,1));
    }
    append_go_pos_step_node(foot_midcoords_list_kuro.front(), get_support_leg_type_list_from_footstep_node_list(footstep_node_list_list_kuro.back()).front());
  };

  void gait_generator::calc_next_coords_velocity_mode (std::vector< std::vector<coordinates> >& ret_list_kuro, const size_t idx)
  {
    std::vector<coordinates> foot_midcoords_list_kuro;
    std::vector<hrp::Vector3> trans_list_kuro;
    std::vector<double> dth_list_kuro;
    calc_foot_midcoords_trans_vector_velocity_mode(foot_midcoords_list_kuro, trans_list_kuro, dth_list_kuro, footstep_node_list_list_kuro[idx-1]);


    for (size_t i = 0; i < 3; i++) {
      std::vector<coordinates> ret;
      for (size_t j = 0; j < foot_midcoords_list_kuro.size(); j++) {
        ret.push_back(foot_midcoords_list_kuro[j]);
        if ( velocity_mode_flg != VEL_ENDING ) {
          ret[j].pos += ret[j].rot * trans_list_kuro[j];
          ret[j].rotate(dth_list_kuro[j], hrp::Vector3(0,0,1));
        }
        /* biped assumed */
        ret[j].pos += ret[j].rot * footstep_param.leg_default_translate_pos[(footstep_node_list_list_kuro[idx-1][0].l_r == RLEG) ? (1 + i)%2 : i%2];
      }
      ret_list_kuro.push_back(ret);
    }
  };

  void gait_generator::overwrite_refzmp_queue(const std::vector< std::vector<step_node> >& fnll_kuro)
  {
    /* clear footstep and refzmp after footstep_index + 1, it means we do not modify current step */
    size_t idx = lcg.get_footstep_index() + 1;
    footstep_node_list_list_kuro.erase(footstep_node_list_list_kuro.begin()+idx, footstep_node_list_list_kuro.end());

    /* add new next steps ;; the number of next steps is fnl.size() */
    footstep_node_list_list_kuro.insert(footstep_node_list_list_kuro.end(), fnll_kuro.begin(), fnll_kuro.end());

    /* remove refzmp after idx for allocation of new refzmp by push_refzmp_from_footstep_list */
    rg.remove_refzmp_cur_list_over_length(idx);
    /* remove refzmp in preview contoroller queue */
    preview_controller_ptr->remove_preview_queue(lcg.get_lcg_count());

    /* reset index and counter */
    rg.set_indices(idx);
    rg.set_refzmp_count(static_cast<size_t>(fnll_kuro[0][0].step_time/dt));
    lcg.set_swing_support_list_list_kuro(footstep_node_list_list_kuro);
    /* reset refzmp */
    for (size_t i = 0; i < fnll_kuro.size(); i++) {
        if (emergency_flg == EMERGENCY_STOP)
            rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list_list_kuro[idx+i],
                                                       lcg.get_swing_leg_dst_coords_list_kuro_idx(footstep_node_list_list_kuro.size()-1),
                                                       lcg.get_support_leg_coords_list_kuro_idx(footstep_node_list_list_kuro.size()-1));
        else {
            if (i==fnll_kuro.size()-1) {
                rg.push_refzmp_from_footstep_list_for_dual(footstep_node_list_list_kuro[fnll_kuro.size()-1],
                                                           lcg.get_swing_leg_dst_coords_list_kuro_idx(footstep_node_list_list_kuro.size()-1),
                                                           lcg.get_support_leg_coords_list_kuro_idx(footstep_node_list_list_kuro.size()-1));
            } else {
                rg.push_refzmp_from_footstep_list_for_single(footstep_node_list_list_kuro[idx+i], lcg.get_support_leg_coords_list_kuro_idx(idx+i));
            }
        }
    }
    /* fill preview controller queue by new refzmp */
    hrp::Vector3 rzmp;
    std::vector<hrp::Vector3> sfzo_list_kuro;
    bool not_solved = true;
    while (not_solved) {
      bool refzmp_exist_p = rg.get_current_refzmp(rzmp, sfzo_list_kuro, default_double_support_ratio, default_double_support_static_ratio);
      /* only biped */
      not_solved = !preview_controller_ptr->update(refzmp, cog, swing_foot_zmp_offset_list_kuro[0], rzmp, sfzo_list_kuro[0], refzmp_exist_p);
      rg.update_refzmp(footstep_node_list_list_kuro);
    }
  };
}

