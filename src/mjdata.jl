

@enum mjtWarning mjWARN_INERTIA = (UInt32)(0) mjWARN_CONTACTFULL = (UInt32)(1) mjWARN_CNSTRFULL = (UInt32)(2) mjWARN_VGEOMFULL = (UInt32)(3) mjWARN_BADQPOS = (UInt32)(4) mjWARN_BADQVEL = (UInt32)(5) mjWARN_BADQACC = (UInt32)(6) mjWARN_BADCTRL = (UInt32)(7) mjNWARNING = (UInt32)(8)
const _mjNWARNING = Integer(mjNWARNING)

@enum mjtTimer mjTIMER_STEP = (UInt32)(0) mjTIMER_FORWARD = (UInt32)(1) mjTIMER_INVERSE = (UInt32)(2) mjTIMER_POSITION = (UInt32)(3) mjTIMER_VELOCITY = (UInt32)(4) mjTIMER_ACTUATION = (UInt32)(5) mjTIMER_ACCELERATION = (UInt32)(6) mjTIMER_CONSTRAINT = (UInt32)(7) mjTIMER_POS_KINEMATICS = (UInt32)(8) mjTIMER_POS_INERTIA = (UInt32)(9) mjTIMER_POS_COLLISION = (UInt32)(10) mjTIMER_POS_MAKE = (UInt32)(11) mjTIMER_POS_PROJECT = (UInt32)(12) mjNTIMER = (UInt32)(13)
const _mjNTIMER = Integer(mjNTIMER)


#struct _mjContact
immutable _mjContact
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

const mjContact = _mjContact

immutable _mjWarningStat
   lastinfo::Cint
   number::Cint
end
const mjWarningStat = _mjWarningStat


immutable _mjTimerStat
   duration::mjtNum
   number::Cint
end
const mjTimerStat = _mjTimerStat


immutable _mjSolverStat
   improvement::mjtNum
   gradient::mjtNum
   lineslope::mjtNum
   nactive::Cint
   nchange::Cint
   neval::Cint
   nupdate::Cint
end
const mjSolverStat = _mjSolverStat

#mutable struct _mjData
type _mjData
   nstack::Cint
   nbuffer::Cint
   pstack::Cint
   maxuse_stack::Cint
   maxuse_con::Cint
   maxuse_efc::Cint

   warning::SVector{8, mjWarningStat}
   timer::SVector{13, mjTimerStat}
   solver::SVector{mjNSOLVER, mjSolverStat}

   solver_iter::Cint
   solver_nnz::Cint
   solver_fwdinv::SVector{2, mjtNum}

   ne::Cint
   nf::Cint
   nefc::Cint
   ncon::Cint
   time::mjtNum
   energy::SVector{2, mjtNum}

   buffer::Ptr{Void}
   stack::Ptr{mjtNum}

   qpos::Ptr{mjtNum}
   qvel::Ptr{mjtNum}
   act::Ptr{mjtNum}
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
   ten_length::Ptr{mjtNum}
   ten_moment::Ptr{mjtNum}
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
   efc_J_colind::Ptr{Cint}
   efc_JT_rownnz::Ptr{Cint}
   efc_JT_rowadr::Ptr{Cint}
   efc_JT_colind::Ptr{Cint}
   efc_solref::Ptr{mjtNum}
   efc_solimp::Ptr{mjtNum}
   efc_margin::Ptr{mjtNum}
   efc_frictionloss::Ptr{mjtNum}
   efc_pos::Ptr{mjtNum}
   efc_J::Ptr{mjtNum}
   efc_JT::Ptr{mjtNum}
   efc_diagApprox::Ptr{mjtNum}
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
   qacc_warmstart::Ptr{mjtNum}

   qfrc_inverse::Ptr{mjtNum}

   cacc::Ptr{mjtNum}
   cfrc_int::Ptr{mjtNum}
   cfrc_ext::Ptr{mjtNum}
end
const mjData = _mjData

# Callback function types TODO

const mjfGeneric = Ptr{Void}
const mjfConFilt = Ptr{Void}
const mjfSensor = Ptr{Void}
const mjfTime = Ptr{Void}
const mjfAct = Ptr{Void}
const mjfSolImp = Ptr{Void}
const mjfSolRef = Ptr{Void}
const mjfCollision = Ptr{Void}


