

@enum mjtWarning WARN_INERTIA = (UInt32)(0) WARN_CONTACTFULL = (UInt32)(1) WARN_CNSTRFULL = (UInt32)(2) WARN_VGEOMFULL = (UInt32)(3) WARN_BADQPOS = (UInt32)(4) WARN_BADQVEL = (UInt32)(5) WARN_BADQACC = (UInt32)(6) WARN_BADCTRL = (UInt32)(7) NWARNING = (UInt32)(8)

@enum mjtTimer mjtIMER_STEP = (UInt32)(0) mjtIMER_FORWARD = (UInt32)(1) mjtIMER_INVERSE = (UInt32)(2) mjtIMER_POSITION = (UInt32)(3) mjtIMER_VELOCITY = (UInt32)(4) mjtIMER_ACTUATION = (UInt32)(5) mjtIMER_ACCELERATION = (UInt32)(6) mjtIMER_CONSTRAINT = (UInt32)(7) mjtIMER_POS_KINEMATICS = (UInt32)(8) mjtIMER_POS_INERTIA = (UInt32)(9) mjtIMER_POS_COLLISION = (UInt32)(10) mjtIMER_POS_MAKE = (UInt32)(11) mjtIMER_POS_PROJECT = (UInt32)(12) NTIMER = (UInt32)(13)


struct mjContact
   dist::mjtNum
   pos::SVector{3, mjtNum}
   frame::SVector{9, mjtNum}
   includemargin::mjtNum
   friction::SVector{5, mjtNum}
   solref::SVector{2, mjtNum}
   solimp::SVector{3, mjtNum}
   mu::mjtNum
   H::SVector{36, mjtNum}
   dim::Cint
   geom1::Cint
   geom2::Cint
   exclude::Cint
   efc_address::Cint
end

struct mjWarningStat
   lastinfo::Cint
   number::Cint
end

struct mjTimerStat
   duration::mjtNum
   number::Cint
end

struct mjSolverStat
   improvement::mjtNum
   gradient::mjtNum
   lineslope::mjtNum
   nactive::Cint
   nchange::Cint
   neval::Cint
   nupdate::Cint
end

struct mjData # might need to be mutable for setting stack pointer
   nstack::Cint
   nbuffer::Cint

   pstack::Cint

   maxuse_stack::Cint
   maxuse_con::Cint
   maxuse_efc::Cint

   warning::SVector{8, mjWarningStat}
   timer::SVector{13, mjTimerStat}
   solver::SVector{NSOLVER, mjSolverStat}
   solver_iter::Cint
   solver_nnz::Cint
   solver_fwdinv::SVector{2, mjtNum}

   ne::Cint
   nf::Cint
   nefc::Cint
   ncon::Cint
   time::mjtNum
   energy::SVector{2, mjtNum}

   buffer::Ptr{Cvoid}
   stack::Ptr{mjtNum}

   qpos::Ptr{mjtNum}
   qvel::Ptr{mjtNum}
   act::Ptr{mjtNum}
   qacc_warmstart::Ptr{mjtNum}
   ctrl::Ptr{mjtNum}
   qfrc_applied::Ptr{mjtNum}
   xfrc_applied::Ptr{mjtNum}
   qacc::Ptr{mjtNum}
   act_dot::Ptr{mjtNum}
   mocap_pos::Ptr{mjtNum}
   mocap_quat::Ptr{mjtNum}
   userdata::Ptr{mjtNum}
   sensordata::Ptr{mjtNum}

   xpos::Ptr{mjtNum}
   xquat::Ptr{mjtNum}
   xmat::Ptr{mjtNum}
   xipos::Ptr{mjtNum}
   ximat::Ptr{mjtNum}
   xanchor::Ptr{mjtNum}
   xaxis::Ptr{mjtNum}
   geom_xpos::Ptr{mjtNum}
   geom_xmat::Ptr{mjtNum}
   site_xpos::Ptr{mjtNum}
   site_xmat::Ptr{mjtNum}
   cam_xpos::Ptr{mjtNum}
   cam_xmat::Ptr{mjtNum}
   light_xpos::Ptr{mjtNum}
   light_xdir::Ptr{mjtNum}

   subtree_com::Ptr{mjtNum}
   cdof::Ptr{mjtNum}
   cinert::Ptr{mjtNum}
   ten_wrapadr::Ptr{Cint}
   ten_wrapnum::Ptr{Cint}
   ten_J_rownnz::Ptr{Cint}
   ten_J_rowadr::Ptr{Cint}
   ten_J_colind::Ptr{Cint}
   ten_length::Ptr{mjtNum}
   ten_J::Ptr{mjtNum}
   wrap_obj::Ptr{Cint}
   wrap_xpos::Ptr{mjtNum}
   actuator_length::Ptr{mjtNum}
   actuator_moment::Ptr{mjtNum}

   crb::Ptr{mjtNum}
   qM::Ptr{mjtNum}
   qLD::Ptr{mjtNum}
   qLDiagInv::Ptr{mjtNum}
   qLDiagSqrtInv::Ptr{mjtNum}

   contact::Ptr{mjContact}

   efc_type::Ptr{Cint}
   efc_id::Ptr{Cint}
   efc_J_rownnz::Ptr{Cint}
   efc_J_rowadr::Ptr{Cint}
   efc_J_rowsuper::Ptr{Cint}
   efc_J_colind::Ptr{Cint}
   efc_JT_rownnz::Ptr{Cint}
   efc_JT_rowadr::Ptr{Cint}
   efc_JT_rowsuper::Ptr{Cint}
   efc_JT_colind::Ptr{Cint}
   efc_J::Ptr{mjtNum}
   efc_JT::Ptr{mjtNum}
   efc_pos::Ptr{mjtNum}
   efc_margin::Ptr{mjtNum}
   efc_frictionloss::Ptr{mjtNum}
   efc_diagApprox::Ptr{mjtNum}
   efc_KBIP::Ptr{mjtNum}
   efc_D::Ptr{mjtNum}
   efc_R::Ptr{mjtNum}

   efc_AR_rownnz::Ptr{Cint}
   efc_AR_rowadr::Ptr{Cint}
   efc_AR_colind::Ptr{Cint}
   efc_AR::Ptr{mjtNum}

   ten_velocity::Ptr{mjtNum}
   actuator_velocity::Ptr{mjtNum}
   cvel::Ptr{mjtNum}
   cdof_dot::Ptr{mjtNum}
   qfrc_bias::Ptr{mjtNum}
   qfrc_passive::Ptr{mjtNum}
   efc_vel::Ptr{mjtNum}
   efc_aref::Ptr{mjtNum}
   subtree_linvel::Ptr{mjtNum}
   subtree_angmom::Ptr{mjtNum}
   actuator_force::Ptr{mjtNum}
   qfrc_actuator::Ptr{mjtNum}
   qfrc_unc::Ptr{mjtNum}
   qacc_unc::Ptr{mjtNum}

   efc_b::Ptr{mjtNum}
   efc_force::Ptr{mjtNum}
   efc_state::Ptr{Cint}
   qfrc_constraint::Ptr{mjtNum}

   qfrc_inverse::Ptr{mjtNum}

   cacc::Ptr{mjtNum}
   cfrc_int::Ptr{mjtNum}
   cfrc_ext::Ptr{mjtNum}
end

# Callback function types TODO

const mjfGeneric = Ptr{Cvoid}
const mjfConFilt = Ptr{Cvoid}
const mjfSensor = Ptr{Cvoid}
const mjfTime = Ptr{Cvoid}
const mjfAct = Ptr{Cvoid}
const mjfCollision = Ptr{Cvoid}


