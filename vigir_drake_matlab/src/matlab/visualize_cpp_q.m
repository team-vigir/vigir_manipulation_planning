% cpp
joint_names = {'back_bkz', 'back_bky', 'back_bkx', 'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2', 'neck_ry', 'hokuyo_joint', 'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx'};
joint_positions = [-0.663225, -0.219388, -0.523599, -0.9407577707095917, -0.34849851607518023, 0.0, 0.0, -1.047156755315536, -1.0566799051482445, -1.156231709378183, -0.02154427580535412, 0.0, -0.018960529272912925, -0.012469013240020655, 0.002421531910259212, -0.0007075490437411091, 5.668869731514367e-05, -4.426976068123295e-05, -9.35381188421917e-06, 0.1780506902255654, 0.2973764408201877, 0.1068069416336173, 0.00020256000762585797, -0.12699556658501676, -0.16799487842425828, 0.174358, 0.14880404123472907, 0.031373741983101625, 0.06317893791807039, -0.11438260856905476, -0.02036183156166611];
base_translation = [0.0736282946142,-0.0806386565929,0.93090613635];
base_rotation = [0.99373027084, -0.0649035673653, 0.00460476052574, -0.0909201404564];  


% matlab q0
%joint_names = {'back_bkz','back_bky','back_bkx','l_arm_shz','l_leg_hpz','l_leg_hpx','l_leg_hpy','l_leg_kny','l_leg_aky','l_leg_akx','l_arm_shx','l_arm_ely','l_arm_elx','l_arm_wry','l_arm_wrx','l_arm_wry2','r_arm_shz','r_leg_hpz','r_leg_hpx','r_leg_hpy','r_leg_kny','r_leg_aky','r_leg_akx','r_arm_shx','r_arm_ely','r_arm_elx','r_arm_wry','r_arm_wrx','r_arm_wry2','neck_ry','hokuyo_joint'};
%joint_positions = [-4.8416e-05,0.0015895,-0.00018978,-0.25412,-0.0098132,0.07576,-0.46319,0.9404,-0.45157,-0.07552,-1.3443,1.969,0.47574,0.0095753,-0.00085321,0.022117,0.29293,0.0098817,-0.07571,-0.46248,0.93898,-0.45089,0.076813,1.3629,1.8535,-0.48797,0.0096058,0.00084978,-0.025142,-0.021544,0];
%base_translation = [-0.010963, 0.00053434, 0.87919];
%base_rotation = [-0.0006862, -0.025832, -7.1323e-05];

% matlab q_sol
%joint_names = {'back_bkz', 'back_bky', 'back_bkx', 'l_arm_shz', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_wry', 'l_arm_wrx', 'l_arm_wry2', 'neck_ry', 'hokuyo_joint', 'r_arm_shz', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_wry', 'r_arm_wrx', 'r_arm_wry2', 'l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky', 'l_leg_akx', 'r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky', 'r_leg_akx'};
%joint_positions = [-0.663225, 0.11948569318544153, -0.523599, -1.3325964398167325, -0.0869329986238263, 0.0, 0.0, -1.230791910982056, -1.0618901002728078, -1.2295371013872614, -0.0018051524939335247, 0.0, -0.10907166769418956, 0.037374966130440106, 0.017833817144387212, 0.0, -0.00020107578834694834, 0.0017707695761586231, 3.659539977122591e-05, 0.027519645160136566, 0.14033552218712522, 0.07867578990679629, 0.04175664242528835, -0.11354498807193876, -0.12100052934977107, 0.04392265653658353, -0.0007348922162639775, -0.11789911776963741, 0.39823035862672873, -0.27342873039216503, 0.020140537794079674];
%base_translation = [0.0587327018176,-0.043410796668,0.932021621897];
%base_rotation = [0.999784766224, -0.00936664631753, -0.00350776225593, -0.0181764344079];
            
visualizer = nodeObject.getVisualizer();
robot_model = nodeObject.getRobotModel();

nq = robot_model.num_positions;
q = zeros(nq,1);

% set normal joints
for i = 1:length(joint_names)
    items = strfind({robot_model.body.jointname}, joint_names{i});
    body_idx = find(~cellfun(@isempty,items));

    joint_idx = robot_model.body(body_idx).position_num;
    q(joint_idx) = joint_positions(i);
end

% set world position
q(1:3) = base_translation;

if ( length(base_rotation) == 4 ) % quat
   q(4:6) = quat2rpy(base_rotation);
elseif ( length(base_rotation) == 3 ) % rpy
   q(4:6) = base_rotation; 
else
    warning('base_rotation has a strange length, ignored');
end

% visualize
visualizer.draw(cputime, q);
