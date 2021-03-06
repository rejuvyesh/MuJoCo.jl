
# TODO overload print/show/display function to not display model and data

struct jlModel
   m::Ptr{mjModel}

   qpos0::Array{mjtNum}
   qpos_spring::Array{mjtNum}
   body_parentid::Array{Cint}
   body_rootid::Array{Cint}
   body_weldid::Array{Cint}
   body_mocapid::Array{Cint}
   body_jntnum::Array{Cint}
   body_jntadr::Array{Cint}
   body_dofnum::Array{Cint}
   body_dofadr::Array{Cint}
   body_geomnum::Array{Cint}
   body_geomadr::Array{Cint}
   body_simple::Array{mjtByte}
   body_sameframe::Array{mjtByte}
   body_pos::Array{mjtNum}
   body_quat::Array{mjtNum}
   body_ipos::Array{mjtNum}
   body_iquat::Array{mjtNum}
   body_mass::Array{mjtNum}
   body_subtreemass::Array{mjtNum}
   body_inertia::Array{mjtNum}
   body_invweight0::Array{mjtNum}
   body_user::Array{mjtNum}
   jnt_type::Array{Cint}
   jnt_qposadr::Array{Cint}
   jnt_dofadr::Array{Cint}
   jnt_bodyid::Array{Cint}
   jnt_group::Array{Cint}
   jnt_limited::Array{mjtByte}
   jnt_solref::Array{mjtNum}
   jnt_solimp::Array{mjtNum}
   jnt_pos::Array{mjtNum}
   jnt_axis::Array{mjtNum}
   jnt_stiffness::Array{mjtNum}
   jnt_range::Array{mjtNum}
   jnt_margin::Array{mjtNum}
   jnt_user::Array{mjtNum}

   dof_bodyid::Array{Cint}
   dof_jntid::Array{Cint}
   dof_parentid::Array{Cint}
   dof_Madr::Array{Cint}
   dof_simplenum::Array{Cint}
   dof_solref::Array{mjtNum}
   dof_solimp::Array{mjtNum}
   dof_frictionloss::Array{mjtNum}
   dof_armature::Array{mjtNum}
   dof_damping::Array{mjtNum}
   dof_invweight0::Array{mjtNum}
   dof_M0::Array{mjtNum}

   geom_type::Array{Cint}
   geom_contype::Array{Cint}
   geom_conaffinity::Array{Cint}
   geom_condim::Array{Cint}
   geom_bodyid::Array{Cint}
   geom_dataid::Array{Cint}
   geom_matid::Array{Cint}
   geom_group::Array{Cint}
   geom_priority::Array{Cint}
   geom_sameframe::Array{mjtByte}
   geom_solmix::Array{mjtNum}
   geom_solref::Array{mjtNum}
   geom_solimp::Array{mjtNum}
   geom_size::Array{mjtNum}
   geom_rbound::Array{mjtNum}
   geom_pos::Array{mjtNum}
   geom_quat::Array{mjtNum}
   geom_friction::Array{mjtNum}
   geom_margin::Array{mjtNum}
   geom_gap::Array{mjtNum}
   geom_user::Array{mjtNum}
   geom_rgba::Array{Cfloat}

   site_type::Array{Cint}
   site_bodyid::Array{Cint}
   site_matid::Array{Cint}
   site_group::Array{Cint}
   site_sameframe::Array{mjtByte}
   site_size::Array{mjtNum}
   site_pos::Array{mjtNum}
   site_quat::Array{mjtNum}
   site_user::Array{mjtNum}
   site_rgba::Array{Cfloat}

   cam_mode::Array{Cint}
   cam_bodyid::Array{Cint}
   cam_targetbodyid::Array{Cint}
   cam_pos::Array{mjtNum}
   cam_quat::Array{mjtNum}
   cam_poscom0::Array{mjtNum}
   cam_pos0::Array{mjtNum}
   cam_mat0::Array{mjtNum}
   cam_fovy::Array{mjtNum}
   cam_ipd::Array{mjtNum}
   cam_user::Array{mjtNum}

   light_mode::Array{Cint}
   light_bodyid::Array{Cint}
   light_targetbodyid::Array{Cint}
   light_directional::Array{mjtByte}
   light_castshadow::Array{mjtByte}
   light_active::Array{mjtByte}
   light_pos::Array{mjtNum}
   light_dir::Array{mjtNum}
   light_poscom0::Array{mjtNum}
   light_pos0::Array{mjtNum}
   light_dir0::Array{mjtNum}
   light_attenuation::Array{Cfloat}
   light_cutoff::Array{Cfloat}
   light_exponent::Array{Cfloat}
   light_ambient::Array{Cfloat}
   light_diffuse::Array{Cfloat}
   light_specular::Array{Cfloat}

   mesh_vertadr::Array{Cint}
   mesh_vertnum::Array{Cint}
   mesh_texcoordadr::Array{Cint}
   mesh_faceadr::Array{Cint}
   mesh_facenum::Array{Cint}
   mesh_graphadr::Array{Cint}
   mesh_vert::Array{Cfloat}
   mesh_normal::Array{Cfloat}
   mesh_texcoord::Array{Cfloat}
   mesh_face::Array{Cint}
   mesh_graph::Array{Cint}

   skin_matid::Array{Cint}
   skin_rgba::Array{Float32}
   skin_inflate::Array{Float32}
   skin_vertadr::Array{Cint}
   skin_vertnum::Array{Cint}
   skin_texcoordadr::Array{Cint}
   skin_faceadr::Array{Cint}
   skin_facenum::Array{Cint}
   skin_boneadr::Array{Cint}
   skin_bonenum::Array{Cint}
   skin_vert::Array{Float32}
   skin_texcoord::Array{Float32}
   skin_face::Array{Cint}
   skin_bonevertadr::Array{Cint}
   skin_bonevertnum::Array{Cint}
   skin_bonebindpos::Array{Float32}
   skin_bonebindquat::Array{Float32}
   skin_bonebodyid::Array{Cint}
   skin_bonevertid::Array{Cint}
   skin_bonevertweight::Array{Float32}

   hfield_size::Array{mjtNum}
   hfield_nrow::Array{Cint}
   hfield_ncol::Array{Cint}
   hfield_adr::Array{Cint}
   hfield_data::Array{Cfloat}

   tex_type::Array{Cint}
   tex_height::Array{Cint}
   tex_width::Array{Cint}
   tex_adr::Array{Cint}
   tex_rgb::Array{mjtByte}

   mat_texid::Array{Cint}
   mat_texuniform::Array{mjtByte}
   mat_texrepeat::Array{Cfloat}
   mat_emission::Array{Cfloat}
   mat_specular::Array{Cfloat}
   mat_shininess::Array{Cfloat}
   mat_reflectance::Array{Cfloat}
   mat_rgba::Array{Cfloat}

   pair_dim::Array{Cint}
   pair_geom1::Array{Cint}
   pair_geom2::Array{Cint}
   pair_signature::Array{Cint}
   pair_solref::Array{mjtNum}
   pair_solimp::Array{mjtNum}
   pair_margin::Array{mjtNum}
   pair_gap::Array{mjtNum}
   pair_friction::Array{mjtNum}

   exclude_signature::Array{Cint}

   eq_type::Array{Cint}
   eq_obj1id::Array{Cint}
   eq_obj2id::Array{Cint}
   eq_active::Array{mjtByte}
   eq_solref::Array{mjtNum}
   eq_solimp::Array{mjtNum}
   eq_data::Array{mjtNum}

   tendon_adr::Array{Cint}
   tendon_num::Array{Cint}
   tendon_matid::Array{Cint}
   tendon_group::Array{Cint}
   tendon_limited::Array{mjtByte}
   tendon_width::Array{mjtNum}
   tendon_solref_lim::Array{mjtNum}
   tendon_solimp_lim::Array{mjtNum}
   tendon_solref_fri::Array{mjtNum}
   tendon_solimp_fri::Array{mjtNum}
   tendon_range::Array{mjtNum}
   tendon_margin::Array{mjtNum}
   tendon_stiffness::Array{mjtNum}
   tendon_damping::Array{mjtNum}
   tendon_frictionloss::Array{mjtNum}
   tendon_lengthspring::Array{mjtNum}
   tendon_length0::Array{mjtNum}
   tendon_invweight0::Array{mjtNum}
   tendon_user::Array{mjtNum}
   tendon_rgba::Array{Cfloat}

   wrap_type::Array{Cint}
   wrap_objid::Array{Cint}
   wrap_prm::Array{mjtNum}

   actuator_trntype::Array{Cint}
   actuator_dyntype::Array{Cint}
   actuator_gaintype::Array{Cint}
   actuator_biastype::Array{Cint}
   actuator_trnid::Array{Cint}
   actuator_group::Array{Cint}
   actuator_ctrllimited::Array{mjtByte}
   actuator_forcelimited::Array{mjtByte}
   actuator_dynprm::Array{mjtNum}
   actuator_gainprm::Array{mjtNum}
   actuator_biasprm::Array{mjtNum}
   actuator_ctrlrange::Array{mjtNum}
   actuator_forcerange::Array{mjtNum}
   actuator_gear::Array{mjtNum}
   actuator_cranklength::Array{mjtNum}
   actuator_acc0::Array{mjtNum}
   actuator_length0::Array{mjtNum}
   actuator_lengthrange::Array{mjtNum}
   actuator_user::Array{mjtNum}

   sensor_type::Array{Cint}
   sensor_datatype::Array{Cint}
   sensor_needstage::Array{Cint}
   sensor_objtype::Array{Cint}
   sensor_objid::Array{Cint}
   sensor_dim::Array{Cint}
   sensor_adr::Array{Cint}
   sensor_cutoff::Array{mjtNum}
   sensor_noise::Array{mjtNum}
   sensor_user::Array{mjtNum}

   numeric_adr::Array{Cint}
   numeric_size::Array{Cint}
   numeric_data::Array{mjtNum}

   text_adr::Array{Cint}
   text_size::Array{Cint}
   text_data::Array{UInt8}

   tuple_adr::Array{Cint}
   tuple_size::Array{Cint}
   tuple_objtype::Array{Cint}
   tuple_objid::Array{Cint}
   tuple_objprm::Array{mjtNum}

   key_time::Array{mjtNum}
   key_qpos::Array{mjtNum}
   key_qvel::Array{mjtNum}
   key_act::Array{mjtNum}

   name_bodyadr::Array{Cint}
   name_jntadr::Array{Cint}
   name_geomadr::Array{Cint}
   name_siteadr::Array{Cint}
   name_camadr::Array{Cint}
   name_lightadr::Array{Cint}
   name_meshadr::Array{Cint}
   name_skinadr::Array{Cint}
   name_hfieldadr::Array{Cint}
   name_texadr::Array{Cint}
   name_matadr::Array{Cint}
   name_pairadr::Array{Cint}
   name_excludeadr::Array{Cint}
   name_eqadr::Array{Cint}
   name_tendonadr::Array{Cint}
   name_actuatoradr::Array{Cint}
   name_sensoradr::Array{Cint}
   name_numericadr::Array{Cint}
   name_textadr::Array{Cint}
   name_tupleadr::Array{Cint}
   name_keyadr::Array{Cint}
   names::Array{UInt8}
end

struct jlData
   d::Ptr{mjData} # point to c struct

   stack::Array{mjtNum}

   qpos::Array{mjtNum}
   qvel::Array{mjtNum}
   act::Array{mjtNum}
   qacc_warmstart::Array{mjtNum}
   ctrl::Array{mjtNum}
   qfrc_applied::Array{mjtNum}
   xfrc_applied::Array{mjtNum}
   qacc::Array{mjtNum}
   act_dot::Array{mjtNum}
   mocap_pos::Array{mjtNum}
   mocap_quat::Array{mjtNum}
   userdata::Array{mjtNum}
   sensordata::Array{mjtNum}

   xpos::Array{mjtNum}
   xquat::Array{mjtNum}
   xmat::Array{mjtNum}
   xipos::Array{mjtNum}
   ximat::Array{mjtNum}
   xanchor::Array{mjtNum}
   xaxis::Array{mjtNum}
   geom_xpos::Array{mjtNum}
   geom_xmat::Array{mjtNum}
   site_xpos::Array{mjtNum}
   site_xmat::Array{mjtNum}
   cam_xpos::Array{mjtNum}
   cam_xmat::Array{mjtNum}
   light_xpos::Array{mjtNum}
   light_xdir::Array{mjtNum}

   subtree_com::Array{mjtNum}
   cdof::Array{mjtNum}
   cinert::Array{mjtNum}
   ten_wrapadr::Array{Cint}
   ten_wrapnum::Array{Cint}
   ten_J_rownnz::Array{Cint}
   ten_J_rowadr::Array{Cint}
   ten_J_colind::Array{Cint}
   ten_length::Array{mjtNum}
   ten_J::Array{mjtNum}
   wrap_obj::Array{Cint}
   wrap_xpos::Array{mjtNum}
   actuator_length::Array{mjtNum}
   actuator_moment::Array{mjtNum}

   crb::Array{mjtNum}
   qM::Array{mjtNum}
   qLD::Array{mjtNum}
   qLDiagInv::Array{mjtNum}
   qLDiagSqrtInv::Array{mjtNum}

   contact::Array{mjContact}

   efc_type::Array{Cint}
   efc_id::Array{Cint}
   efc_J_rownnz::Array{Cint}
   efc_J_rowadr::Array{Cint}
   efc_J_rowsuper::Array{Cint}
   efc_J_colind::Array{Cint}
   efc_JT_rownnz::Array{Cint}
   efc_JT_rowadr::Array{Cint}
   efc_JT_rowsuper::Array{Cint}
   efc_JT_colind::Array{Cint}
   efc_J::Array{mjtNum}
   efc_JT::Array{mjtNum}
   efc_pos::Array{mjtNum}
   efc_margin::Array{mjtNum}
   efc_frictionloss::Array{mjtNum}
   efc_diagApprox::Array{mjtNum}
   efc_KBIP::Array{mjtNum}
   efc_D::Array{mjtNum}
   efc_R::Array{mjtNum}

   efc_AR_rownnz::Array{Cint}
   efc_AR_rowadr::Array{Cint}
   efc_AR_colind::Array{Cint}
   efc_AR::Array{mjtNum}

   ten_velocity::Array{mjtNum}
   actuator_velocity::Array{mjtNum}
   cvel::Array{mjtNum}
   cdof_dot::Array{mjtNum}
   qfrc_bias::Array{mjtNum}
   qfrc_passive::Array{mjtNum}
   efc_vel::Array{mjtNum}
   efc_aref::Array{mjtNum}
   subtree_linvel::Array{mjtNum}
   subtree_angmom::Array{mjtNum}
   actuator_force::Array{mjtNum}
   qfrc_actuator::Array{mjtNum}
   qfrc_unc::Array{mjtNum}
   qacc_unc::Array{mjtNum}

   efc_b::Array{mjtNum}
   efc_force::Array{mjtNum}
   efc_state::Array{Cint}
   qfrc_constraint::Array{mjtNum}

   qfrc_inverse::Array{mjtNum}

   cacc::Array{mjtNum}
   cfrc_int::Array{mjtNum}
   cfrc_ext::Array{mjtNum}
end

# added stack field manually
# manually did second terms in the parantheses
function getdatasize(m::mjModel, d::mjData)
   return Dict(
               :stack=>(d.nstack,1),
               :qpos=>(m.nq,1),
               :qvel=>(m.nv,1),
               :act=>(m.na,1),
               :qacc_warmstart=>(m.nv,1),
               :ctrl=>(m.nu,1),
               :qfrc_applied=>(m.nv,1),
               :xfrc_applied=>(m.nbody,6),
               :qacc=>(m.nv,1),
               :act_dot=>(m.na,1),
               :mocap_pos=>(m.nmocap,3),
               :mocap_quat=>(m.nmocap,4),
               :userdata=>(m.nuserdata,1),
               :sensordata=>(m.nsensordata,1),
               :xpos=>(m.nbody,3),
               :xquat=>(m.nbody,4),
               :xmat=>(m.nbody,9),
               :xipos=>(m.nbody,3),
               :ximat=>(m.nbody,9),
               :xanchor=>(m.njnt,3),
               :xaxis=>(m.njnt,3),
               :geom_xpos=>(m.ngeom,3),
               :geom_xmat=>(m.ngeom,9),
               :site_xpos=>(m.nsite,3),
               :site_xmat=>(m.nsite,9),
               :cam_xpos=>(m.ncam,3),
               :cam_xmat=>(m.ncam,9),
               :light_xpos=>(m.nlight,3),
               :light_xdir=>(m.nlight,3),
               :subtree_com=>(m.nbody,3),
               :cdof=>(m.nv,6),
               :cinert=>(m.nbody,10),
               :ten_wrapadr=>(m.ntendon,1),
               :ten_wrapnum=>(m.ntendon,1),
               :ten_J_rownnz=>(m.ntendon,1),
               :ten_J_rowadr=>(m.ntendon,1),
               :ten_J_colind=>(m.ntendon,m.nv),
               :ten_length=>(m.ntendon,1),
               :ten_J=>(m.ntendon,m.nv),
               :wrap_obj=>(m.nwrap*2,1),
               :wrap_xpos=>(m.nwrap*2,3),
               :actuator_length=>(m.nu,1),
               :actuator_moment=>(m.nu,m.nv),
               :crb=>(m.nbody,10),
               :qM=>(m.nM,1),
               :qLD=>(m.nM,1),
               :qLDiagInv=>(m.nv,1),
               :qLDiagSqrtInv=>(m.nv,1),
               :contact=>(m.nconmax,1),
               :efc_type=>(m.njmax,1),
               :efc_id=>(m.njmax,1),
               :efc_J_rownnz=>(m.njmax,1),
               :efc_J_rowadr=>(m.njmax,1),
               :efc_J_rowsuper=>(m.njmax,1),
               :efc_J_colind=>(m.njmax,m.nv),
               :efc_JT_rownnz=>(m.nv,1),
               :efc_JT_rowadr=>(m.nv,1),
               :efc_JT_rowsuper=>(m.nv,1),
               :efc_JT_colind=>(m.nv,m.njmax),
               :efc_J=>(m.njmax,m.nv),
               :efc_JT=>(m.nv,m.njmax),
               :efc_pos=>(m.njmax,1),
               :efc_margin=>(m.njmax,1),
               :efc_frictionloss=>(m.njmax,1),
               :efc_diagApprox=>(m.njmax,1),
               :efc_KBIP=>(m.njmax,4),
               :efc_D=>(m.njmax,1),
               :efc_R=>(m.njmax,1),
               :efc_AR_rownnz=>(m.njmax,1),
               :efc_AR_rowadr=>(m.njmax,1),
               :efc_AR_colind=>(m.njmax,m.njmax),
               :efc_AR=>(m.njmax,m.njmax),
               :ten_velocity=>(m.ntendon,1),
               :actuator_velocity=>(m.nu,1),
               :cvel=>(m.nbody,6),
               :cdof_dot=>(m.nv,6),
               :qfrc_bias=>(m.nv,1),
               :qfrc_passive=>(m.nv,1),
               :efc_vel=>(m.njmax,1),
               :efc_aref=>(m.njmax,1),
               :subtree_linvel=>(m.nbody,3),
               :subtree_angmom=>(m.nbody,3),
               :actuator_force=>(m.nu,1),
               :qfrc_actuator=>(m.nv,1),
               :qfrc_unc=>(m.nv,1),
               :qacc_unc=>(m.nv,1),
               :efc_b=>(m.njmax,1),
               :efc_force=>(m.njmax,1),
               :efc_state=>(m.njmax,1),
               :qfrc_constraint=>(m.nv,1),
               :qfrc_inverse=>(m.nv,1),
               :cacc=>(m.nbody,6),
               :cfrc_int=>(m.nbody,6),
               :cfrc_ext=>(m.nbody,6)
              )
end

function getmodelsize(m::mjModel)
   return Dict(
               :qpos0=>(m.nq,1),
               :qpos_spring=>(m.nq,1),

               :body_parentid=>(m.nbody,1),
               :body_rootid=>(m.nbody,1),
               :body_weldid=>(m.nbody,1),
               :body_mocapid=>(m.nbody,1),
               :body_jntnum=>(m.nbody,1),
               :body_jntadr=>(m.nbody,1),
               :body_dofnum=>(m.nbody,1),
               :body_dofadr=>(m.nbody,1),
               :body_geomnum=>(m.nbody,1),
               :body_geomadr=>(m.nbody,1),
               :body_simple=>(m.nbody,1),
               :body_sameframe=>(m.nbody,1),
               :body_pos=>(m.nbody,3),
               :body_quat=>(m.nbody,4),
               :body_ipos=>(m.nbody,3),
               :body_iquat=>(m.nbody,4),
               :body_mass=>(m.nbody,1),
               :body_subtreemass=>(m.nbody,1),
               :body_inertia=>(m.nbody,3),
               :body_invweight0=>(m.nbody,2),
               :body_user=>(m.nbody,m.nuser_body),

               :jnt_type=>(m.njnt,1),
               :jnt_qposadr=>(m.njnt,1),
               :jnt_dofadr=>(m.njnt,1),
               :jnt_bodyid=>(m.njnt,1),
               :jnt_group=>(m.njnt,1),
               :jnt_limited=>(m.njnt,1),
               :jnt_solref=>(m.njnt,NREF),
               :jnt_solimp=>(m.njnt,NIMP),
               :jnt_pos=>(m.njnt,3),
               :jnt_axis=>(m.njnt,3),
               :jnt_stiffness=>(m.njnt,1),
               :jnt_range=>(m.njnt,2),
               :jnt_margin=>(m.njnt,1),
               :jnt_user=>(m.njnt,m.nuser_jnt),

               :dof_bodyid=>(m.nv,1),
               :dof_jntid=>(m.nv,1),
               :dof_parentid=>(m.nv,1),
               :dof_Madr=>(m.nv,1),
               :dof_simplenum=>(m.nv,1),
               :dof_solref=>(m.nv,NREF),
               :dof_solimp=>(m.nv,NIMP),
               :dof_frictionloss=>(m.nv,1),
               :dof_armature=>(m.nv,1),
               :dof_damping=>(m.nv,1),
               :dof_invweight0=>(m.nv,1),
               :dof_M0=>(m.nv,1),
               :geom_type=>(m.ngeom,1),
               :geom_contype=>(m.ngeom,1),
               :geom_conaffinity=>(m.ngeom,1),
               :geom_condim=>(m.ngeom,1),
               :geom_bodyid=>(m.ngeom,1),
               :geom_dataid=>(m.ngeom,1),
               :geom_matid=>(m.ngeom,1),
               :geom_group=>(m.ngeom,1),
               :geom_priority=>(m.ngeom,1),
               :geom_sameframe=>(m.ngeom,1),
               :geom_solmix=>(m.ngeom,1),
               :geom_solref=>(m.ngeom,NREF),
               :geom_solimp=>(m.ngeom,NIMP),
               :geom_size=>(m.ngeom,3),
               :geom_rbound=>(m.ngeom,1),
               :geom_pos=>(m.ngeom,3),
               :geom_quat=>(m.ngeom,4),
               :geom_friction=>(m.ngeom,3),
               :geom_margin=>(m.ngeom,1),
               :geom_gap=>(m.ngeom,1),
               :geom_user=>(m.ngeom,m.nuser_geom),
               :geom_rgba=>(m.ngeom,4),

               :site_type=>(m.nsite,1),
               :site_bodyid=>(m.nsite,1),
               :site_matid=>(m.nsite,1),
               :site_group=>(m.nsite,1),
               :site_sameframe=>(m.nsite,1),
               :site_size=>(m.nsite,3),
               :site_pos=>(m.nsite,3),
               :site_quat=>(m.nsite,4),
               :site_user=>(m.nsite,m.nuser_site),
               :site_rgba=>(m.nsite,4),

               :cam_mode=>(m.ncam,1),
               :cam_bodyid=>(m.ncam,1),
               :cam_targetbodyid=>(m.ncam,1),
               :cam_pos=>(m.ncam,3),
               :cam_quat=>(m.ncam,4),
               :cam_poscom0=>(m.ncam,3),
               :cam_pos0=>(m.ncam,3),
               :cam_mat0=>(m.ncam,9),
               :cam_fovy=>(m.ncam,1),
               :cam_ipd=>(m.ncam,1),
               :cam_user=>(m.ncam,m.nuser_cam),

               :light_mode=>(m.nlight,1),
               :light_bodyid=>(m.nlight,1),
               :light_targetbodyid=>(m.nlight,1),
               :light_directional=>(m.nlight,1),
               :light_castshadow=>(m.nlight,1),
               :light_active=>(m.nlight,1),
               :light_pos=>(m.nlight,3),
               :light_dir=>(m.nlight,3),
               :light_poscom0=>(m.nlight,3),
               :light_pos0=>(m.nlight,3),
               :light_dir0=>(m.nlight,3),
               :light_attenuation=>(m.nlight,3),
               :light_cutoff=>(m.nlight,1),
               :light_exponent=>(m.nlight,1),
               :light_ambient=>(m.nlight,3),
               :light_diffuse=>(m.nlight,3),
               :light_specular=>(m.nlight,3),

               :mesh_vertadr=>(m.nmesh,1),
               :mesh_vertnum=>(m.nmesh,1),
               :mesh_texcoordadr=>(m.nmesh,1),
               :mesh_faceadr=>(m.nmesh,1),
               :mesh_facenum=>(m.nmesh,1),
               :mesh_graphadr=>(m.nmesh,1),
               :mesh_vert=>(m.nmeshvert,3),
               :mesh_normal=>(m.nmeshvert,3),
               :mesh_texcoord=>(m.nmeshtexvert,2),
               :mesh_face=>(m.nmeshface,3),
               :mesh_graph=>(m.nmeshgraph,1),

               :skin_matid=>(m.nskin,1),
               :skin_rgba=>(m.nskin,4),
               :skin_inflate=>(m.nskin,1),
               :skin_vertadr=>(m.nskin,1),
               :skin_vertnum=>(m.nskin,1),
               :skin_texcoordadr=>(m.nskin,1),
               :skin_faceadr=>(m.nskin,1),
               :skin_facenum=>(m.nskin,1),
               :skin_boneadr=>(m.nskin,1),
               :skin_bonenum=>(m.nskin,1),
               :skin_vert=>(m.nskinvert,3),
               :skin_texcoord=>(m.nskintexvert,2),
               :skin_face=>(m.nskinface,3),
               :skin_bonevertadr=>(m.nskinbone,1),
               :skin_bonevertnum=>(m.nskinbone,1),
               :skin_bonebindpos=>(m.nskinbone,3),
               :skin_bonebindquat=>(m.nskinbone,4),
               :skin_bonebodyid=>(m.nskinbone,1),
               :skin_bonevertid=>(m.nskinbonevert,1),
               :skin_bonevertweight=>(m.nskinbonevert,1),

               :hfield_size=>(m.nhfield,4),
               :hfield_nrow=>(m.nhfield,1),
               :hfield_ncol=>(m.nhfield,1),
               :hfield_adr=>(m.nhfield,1),
               :hfield_data=>(m.nhfielddata,1),

               :tex_type=>(m.ntex,1),
               :tex_height=>(m.ntex,1),
               :tex_width=>(m.ntex,1),
               :tex_adr=>(m.ntex,1),
               :tex_rgb=>(m.ntexdata,1),

               :mat_texid=>(m.nmat,1),
               :mat_texuniform=>(m.nmat,1),
               :mat_texrepeat=>(m.nmat,2),
               :mat_emission=>(m.nmat,1),
               :mat_specular=>(m.nmat,1),
               :mat_shininess=>(m.nmat,1),
               :mat_reflectance=>(m.nmat,1),
               :mat_rgba=>(m.nmat,4),

               :pair_dim=>(m.npair,1),
               :pair_geom1=>(m.npair,1),
               :pair_geom2=>(m.npair,1),
               :pair_signature=>(m.npair,1),
               :pair_solref=>(m.npair,NREF),
               :pair_solimp=>(m.npair,NIMP),
               :pair_margin=>(m.npair,1),
               :pair_gap=>(m.npair,1),
               :pair_friction=>(m.npair,5),

               :exclude_signature=>(m.nexclude,1),

               :eq_type=>(m.neq,1),
               :eq_obj1id=>(m.neq,1),
               :eq_obj2id=>(m.neq,1),
               :eq_active=>(m.neq,1),
               :eq_solref=>(m.neq,NREF),
               :eq_solimp=>(m.neq,NIMP),
               :eq_data=>(m.neq,NEQDATA),

               :tendon_adr=>(m.ntendon,1),
               :tendon_num=>(m.ntendon,1),
               :tendon_matid=>(m.ntendon,1),
               :tendon_group=>(m.ntendon,1),
               :tendon_limited=>(m.ntendon,1),
               :tendon_width=>(m.ntendon,1),
               :tendon_solref_lim=>(m.ntendon,NREF),
               :tendon_solimp_lim=>(m.ntendon,NIMP),
               :tendon_solref_fri=>(m.ntendon,NREF),
               :tendon_solimp_fri=>(m.ntendon,NIMP),
               :tendon_range=>(m.ntendon,2),
               :tendon_margin=>(m.ntendon,1),
               :tendon_stiffness=>(m.ntendon,1),
               :tendon_damping=>(m.ntendon,1),
               :tendon_frictionloss=>(m.ntendon,1),
               :tendon_lengthspring=>(m.ntendon,1),
               :tendon_length0=>(m.ntendon,1),
               :tendon_invweight0=>(m.ntendon,1),
               :tendon_user=>(m.ntendon,m.nuser_tendon),
               :tendon_rgba=>(m.ntendon,4),

               :wrap_type=>(m.nwrap,1),
               :wrap_objid=>(m.nwrap,1),
               :wrap_prm=>(m.nwrap,1),

               :actuator_trntype=>(m.nu,1),
               :actuator_dyntype=>(m.nu,1),
               :actuator_gaintype=>(m.nu,1),
               :actuator_biastype=>(m.nu,1),
               :actuator_trnid=>(m.nu,2),
               :actuator_group=>(m.nu,1),
               :actuator_ctrllimited=>(m.nu,1),
               :actuator_forcelimited=>(m.nu,1),
               :actuator_dynprm=>(m.nu,NDYN),
               :actuator_gainprm=>(m.nu,NGAIN),
               :actuator_biasprm=>(m.nu,NBIAS),
               :actuator_ctrlrange=>(m.nu,2),
               :actuator_forcerange=>(m.nu,2),
               :actuator_gear=>(m.nu,6),
               :actuator_cranklength=>(m.nu,1),
               :actuator_acc0=>(m.nu,1),
               :actuator_length0=>(m.nu,1),
               :actuator_lengthrange=>(m.nu,2),
               :actuator_user=>(m.nu,m.nuser_actuator),

               :sensor_type=>(m.nsensor,1),
               :sensor_datatype=>(m.nsensor,1),
               :sensor_needstage=>(m.nsensor,1),
               :sensor_objtype=>(m.nsensor,1),
               :sensor_objid=>(m.nsensor,1),
               :sensor_dim=>(m.nsensor,1),
               :sensor_adr=>(m.nsensor,1),
               :sensor_cutoff=>(m.nsensor,1),
               :sensor_noise=>(m.nsensor,1),
               :sensor_user=>(m.nsensor,m.nuser_sensor),

               :numeric_adr=>(m.nnumeric,1),
               :numeric_size=>(m.nnumeric,1),
               :numeric_data=>(m.nnumericdata,1),

               :text_adr=>(m.ntext,1),
               :text_size=>(m.ntext,1),
               :text_data=>(m.ntextdata,1),

               :tuple_adr=>(m.ntuple,1),
               :tuple_size=>(m.ntuple,1),
               :tuple_objtype=>(m.ntupledata,1),
               :tuple_objid=>(m.ntupledata,1),
               :tuple_objprm=>(m.ntupledata,1),

               :key_time=>(m.nkey,1),
               :key_qpos=>(m.nkey,m.nq),
               :key_qvel=>(m.nkey,m.nv),
               :key_act=>(m.nkey,m.na),

               :name_bodyadr=>(m.nbody,1),
               :name_jntadr=>(m.njnt,1),
               :name_geomadr=>(m.ngeom,1),
               :name_siteadr=>(m.nsite,1),
               :name_camadr=>(m.ncam,1),
               :name_lightadr=>(m.nlight,1),
               :name_meshadr=>(m.nmesh,1),
               :name_skinadr=>(m.nskin,1),
               :name_hfieldadr=>(m.nhfield,1),
               :name_texadr=>(m.ntex,1),
               :name_matadr=>(m.nmat,1),
               :name_pairadr=>(m.npair,1),
               :name_excludeadr=>(m.nexclude,1),
               :name_eqadr=>(m.neq,1),
               :name_tendonadr=>(m.ntendon,1),
               :name_actuatoradr=>(m.nu,1),
               :name_sensoradr=>(m.nsensor,1),
               :name_numericadr=>(m.nnumeric,1),
               :name_textadr=>(m.ntext,1),
               :name_tupleadr=>(m.ntuple,1),
               :name_keyadr=>(m.nkey,1),
               :names=>(m.nnames,1)
              )
end

