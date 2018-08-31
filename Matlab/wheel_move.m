function [output,state_Num] = wheel_move(wheel_speeds,vehicle_Params,sym_Params,tire_Params,wheel_location,slipAngle,lateralLoad,laststate)
    
    % INIT
    x_CC = [];    y_CC = [];    x_FC = [];    y_FC = [];    R_FC = [];
    sigma_CG = [];    v_tan = [];    omega = [];    d_psi = [];
    acp_CG = [];    acp_psi = [];
    dm_lateral_front = [];    dm_lateral_rear = [];
    dm_longitudinal_front = [];    dm_longitudinal_rear = [];
                
    x_start = wheel_location(1);
    y_start = wheel_location(2);

    L = vehicle_Params(1);
    T_front = vehicle_Params(2);
    T_rear = vehicle_Params(3);
    CG_height = vehicle_Params(4);
    a = vehicle_Params(5);
    b = vehicle_Params(6);
    mass = vehicle_Params(7);
    m1 = vehicle_Params(8);
    m2 = vehicle_Params(9);
    m3 = vehicle_Params(10);
    m4 = vehicle_Params(11);
    psi = vehicle_Params(12);

    steer = sym_Params(1);
    speed = sym_Params(2);
    max_steer = sym_Params(3);
    max_speed = sym_Params(4);
    C_alpha = sym_Params(5);
    g = sym_Params(6);
    dt = sym_Params(7);

    beta_FL = tire_Params(1);
    beta_FR = tire_Params(2);
    beta_RL = tire_Params(3);
    beta_RR = tire_Params(4);
    
    m1_s = mass*(b/L)/2;         % [kg]
    m2_s = mass*(b/L)/2;         % [kg]
    m3_s = mass*(a/L)/2;         % [kg]
    m4_s = mass*(a/L)/2;         % [kg]

    % TELEMETRY
    v = speed*1000/3600;
    delta = steer*pi/180;

    sigma_FL = 0;    
    sigma_FR = 0;
    sigma_RR = 0;    
    sigma_RL = 0;

    k_F = 0;        n_F = 0;
    k_R = 0;        n_R = 0;

    x_CG = wheel_location(1);
    y_CG = wheel_location(2);

    x_FR = wheel_location(3);
    y_FR = wheel_location(4);

    x_RR = wheel_location(5);
    y_RR = wheel_location(6);

    x_FL = wheel_location(7);
    y_FL = wheel_location(8);

    x_RL = wheel_location(9);
    y_RL = wheel_location(10);

    v_frontLeft  = wheel_speeds(1);
    v_frontRight = wheel_speeds(2);
    v_rearLeft  = wheel_speeds(3);
    v_rearRight  = wheel_speeds(4);

    dw = 20;

    wheel_speeds = [wheel_speeds; wheel_speeds];

    counter = 1;

        if(delta(counter)>0 || laststate==1)
            if(T_front>=T_rear)
                x_FR = L;    y_FR = 0;
                x_RR = 0;    y_RR = (T_front-T_rear)/2;

                x_FL = L;    y_FL = T_front;
                x_RL = 0;    y_RL = T_rear+(T_front-T_rear)/2;

                x_G = b;    y_G = T_front/2;
            else
                x_FR = L;    y_FR = (T_rear-T_front)/2;
                x_RR = 0;    y_RR = 0;                

                x_FL = L;    y_FL = T_front+(T_front-T_rear)/2;
                x_RL = 0;    y_RL = T_rear;

                x_G = b;    y_G = T_rear/2;
            end
            
            state = 'left'; 
            if(laststate~=0) 
                state_Num = 1;
            else
                state_Num = 0;
            end
                m_FL = m1(counter);
                m_FR = m2(counter);
                m_RL = m3(counter);
                m_RR = m4(counter);

                sigma_FL(counter) = delta(counter)-beta_FL(counter);    
                sigma_FR(counter) = delta(counter)-beta_FR(counter);    
                sigma_RL(counter) = -beta_RL(counter);
                sigma_RR(counter) = -beta_RR(counter);
                
                k_R(counter) = tan(sigma_RR(counter)-pi/2);                
                n_R(counter) = y_RR-x_RR*k_R(counter);

                k_F(counter) = tan(sigma_FR(counter)-pi/2);
                n_F(counter) = y_FR-x_FR*k_F(counter);

                x_CC(counter) = (n_F(counter)-n_R(counter))/(k_R(counter)-k_F(counter));
                y_CC(counter) = x_CC(counter)*k_F(counter)+n_F(counter);

                x_FC(counter) = x_CC(counter);
                y_FC(counter) = y_CG(counter);

                R_FC(counter) = y_FC(counter)-y_CC(counter);
                sigma_CG(counter) = atan((x_G-x_CC)/(y_CC-y_G));
                

                R_FR = sqrt((y_FR-y_CC(counter))^2+(x_FR-x_CC(counter))^2);
                R_RR = sqrt((y_RR-y_CC(counter))^2+(x_RR-x_CC(counter))^2);

                R_FL = sqrt((y_FL-y_CC(counter))^2+(x_FL-x_CC(counter))^2);
                R_RL = sqrt((y_RL-y_CC(counter))^2+(x_RL-x_CC(counter))^2);

                R_CG = sqrt((y_G-y_CC(counter))^2+(x_G-x_CC(counter))^2);

                %v_tan(counter) = v_frontRight*R_CG/R_FR;

                v_tan(counter) = v/cos(sigma_CG(counter));
                omega(counter) = v_tan(counter)/R_CG;
                d_psi(counter) = omega(counter)*dt;
                d_S = R_CG*d_psi(counter);
                dx_CG = d_S*cos(sigma_CG(counter)+d_psi(counter));
                dy_CG = d_S*sin(sigma_CG(counter)+d_psi(counter));

                v_frontLeft = v_tan(counter)*R_FL/R_CG;
                v_rearLeft = v_tan(counter)*R_RL/R_CG;

                v_frontRight = v_tan(counter)*R_FR/R_CG;
                v_rearRight = v_tan(counter)*R_RR/R_CG;

                F_fl = (m_FL*v_frontLeft^2/R_FL);
                if(F_fl>=min(lateralLoad) && F_fl<=max(lateralLoad))
                    position = find(lateralLoad<=F_fl);
                    slipFrontLeft = slipAngle(position(end));
                else if(F_fl>max(lateralLoad))
                    slipFrontLeft = max(slipAngle);
                     else
                        slipFrontLeft = min(slipAngle);
                     end
                end
                beta_FL(counter+1) = slipFrontLeft*pi/180;

                F_fr = (m_FR*v_frontRight^2/R_FR);
                if(F_fr>=min(lateralLoad) && F_fr<=max(lateralLoad))
                    position = find(lateralLoad<=F_fr);
                    slipFrontRight = slipAngle(position(end));
                else if(F_fr>max(lateralLoad))
                    slipFrontRight = max(slipAngle);
                     else
                        slipFrontRight = min(slipAngle);
                     end
                end
                beta_FR(counter+1) = slipFrontRight*pi/180;

                F_rl = (m_RL*v_rearLeft^2/R_RL);
                if(F_rl>=min(lateralLoad) && F_rl<=max(lateralLoad))
                    position = find(lateralLoad<=F_rl);
                    slipRearLeft = slipAngle(position(end));
                else if(F_rl>max(lateralLoad))
                    slipRearLeft = max(slipAngle);
                     else
                        slipRearLeft = min(slipAngle);
                     end
                end
                beta_RL(counter+1) = slipRearLeft*pi/180;

                F_rr = (m_RR*v_rearRight^2/R_RR);
                if(F_rr>=min(lateralLoad) && F_rr<=max(lateralLoad))
                    position = find(lateralLoad<=F_rr);
                    slipRearRight = slipAngle(position(end));
                else if(F_rr>max(lateralLoad))
                    slipRearRight = max(slipAngle);
                     else
                        slipRearRight = min(slipAngle);
                     end
                end
                beta_RR(counter+1) = slipRearRight*pi/180;

            psi(counter+1) = psi(counter)+d_psi(counter);
            x_CG(counter+1) = x_CG(counter)+dx_CG*cos(psi(counter+1))-dy_CG*sin(psi(counter+1));
            y_CG(counter+1) = y_CG(counter)+dx_CG*sin(psi(counter+1))+dy_CG*cos(psi(counter+1));

            x_FR(counter+1) = x_CG(counter+1)+sqrt(a^2+(T_front/2)^2)*cos(psi(counter+1)-atan((T_front/2)/a));
            y_FR(counter+1) = y_CG(counter+1)+sqrt(a^2+(T_front/2)^2)*sin(psi(counter+1)-atan((T_front/2)/a));

            % x_RR(counter+1) = x_FR(counter+1)-L*cos(psi(counter+1));
            % y_RR(counter+1) = y_FR(counter+1)-L*sin(psi(counter+1));

            x_RR(counter+1) = x_CG(counter+1)-b*cos(psi(counter+1))+(T_rear/2)*sin(psi(counter+1));
            y_RR(counter+1) = y_CG(counter+1)-b*sin(psi(counter+1))-(T_rear/2)*cos(psi(counter+1));

            x_FL(counter+1) = x_FR(counter+1)-T_front*sin(psi(counter+1));
            y_FL(counter+1) = y_FR(counter+1)+T_front*cos(psi(counter+1));

            x_RL(counter+1) = x_RR(counter+1)-T_rear*sin(psi(counter+1));
            y_RL(counter+1) = y_RR(counter+1)+T_rear*cos(psi(counter+1));

        else if(delta(counter)<0 || laststate==2)
                
                if(T_front>=T_rear)
                    x_FR = L;    y_FR = 0;
                    x_RR = 0;    y_RR = (T_front-T_rear)/2;

                    x_FL = L;    y_FL = T_front;
                    x_RL = 0;    y_RL = T_rear+(T_front-T_rear)/2;

                    x_G = b;    y_G = T_front/2;
                else
                    x_FR = L;    y_FR = (T_rear-T_front)/2;
                    x_RR = 0;    y_RR = 0;                

                    x_FL = L;    y_FL = T_front+(T_front-T_rear)/2;
                    x_RL = 0;    y_RL = T_rear;

                    x_G = b;    y_G = T_rear/2;
                end

                state = 'right';
                if(laststate~=0)
                    state_Num = 2;
                else
                    state_Num = 0;
                end
                    m_FL = m1(counter);
                    m_FR = m2(counter);
                    m_RL = m3(counter);
                    m_RR = m4(counter);

                    sigma_FL(counter) = delta(counter)+beta_FL(counter);    
                    sigma_RL(counter) = beta_RL(counter);

                    k_R(counter) = tan(sigma_RL(counter)-pi/2);
                    n_R(counter) = y_RL-x_RL*k_R(counter);

                    k_F(counter) = tan(sigma_FL(counter)-pi/2);
                    n_F(counter) = y_FL-x_FL*k_F(counter);

                    x_CC(counter) = (n_F(counter)-n_R(counter))/(k_R(counter)-k_F(counter));
                    y_CC(counter) = x_CC(counter)*k_F(counter)+n_F(counter);

                    x_FC(counter) = x_CC(counter);
                    y_FC(counter) = y_CG(counter);

                    R_FC(counter) = y_FC(counter)-y_CC(counter);
                    sigma_CG(counter) = atan((x_G(counter)-x_CC(counter))/(y_CC(counter)-y_G(counter)));

                    R_FL = sqrt((y_FL-y_CC(counter))^2+(x_FL-x_CC(counter))^2);
                    R_RL = sqrt((y_RL-y_CC(counter))^2+(x_RL-x_CC(counter))^2);

                    R_FR = sqrt((y_FR-y_CC(counter))^2+(x_FR-x_CC(counter))^2);
                    R_RR = sqrt((y_RR-y_CC(counter))^2+(x_RR-x_CC(counter))^2);

                    R_CG = sqrt((y_G-y_CC(counter))^2+(x_G-x_CC(counter))^2);

                    v_tan(counter) = v/cos(sigma_CG(counter));
                    omega(counter) = v_tan(counter)/R_CG;
                    d_psi(counter) = -omega(counter)*dt;
                    d_S = R_CG*d_psi(counter); %*sign(delta(counter));
                    dx_CG = -d_S*cos(sigma_CG(counter)+d_psi(counter));
                    dy_CG = d_S*sin(sigma_CG(counter)+d_psi(counter));

                    v_frontLeft = v_tan(counter)*R_FL/R_CG;
                    v_rearLeft = v_tan(counter)*R_RL/R_CG;

                    v_frontRight = v_tan(counter)*R_FR/R_CG;
                    v_rearRight = v_tan(counter)*R_RR/R_CG;

                    F_fl = (m_FL*v_frontLeft^2/R_FL);
                    if(F_fl>=min(lateralLoad) && F_fl<=max(lateralLoad))
                        position = find(lateralLoad<=F_fl);
                        slipFrontLeft = slipAngle(position(end));
                    else if(F_fl>max(lateralLoad))
                        slipFrontLeft = max(slipAngle);
                         else
                            slipFrontLeft = min(slipAngle);
                         end
                    end
                    beta_FL(counter+1) = slipFrontLeft*pi/180;

                    F_fr = (m_FR*v_frontRight^2/R_FR);
                    if(F_fr>=min(lateralLoad) && F_fr<=max(lateralLoad))
                        position = find(lateralLoad<=F_fr);
                        slipFrontRight = slipAngle(position(end));
                    else if(F_fr>max(lateralLoad))
                        slipFrontRight = max(slipAngle);
                         else
                            slipFrontRight = min(slipAngle);
                         end
                    end
                    beta_FR(counter+1) = slipFrontRight*pi/180;

                    F_rl = (m_RL*v_rearLeft^2/R_RL);
                    if(F_rl>=min(lateralLoad) && F_rl<=max(lateralLoad))
                        position = find(lateralLoad<=F_rl);
                        slipRearLeft = slipAngle(position(end));
                    else if(F_rl>max(lateralLoad))
                        slipRearLeft = max(slipAngle);
                         else
                            slipRearLeft = min(slipAngle);
                         end
                    end
                    beta_RL(counter+1) = slipRearLeft*pi/180;

                    F_rr = (m_RR*v_rearRight^2/R_RR);
                    if(F_rr>=min(lateralLoad) && F_rr<=max(lateralLoad))
                        position = find(lateralLoad<=F_rr);
                        slipRearRight = slipAngle(position(end));
                    else if(F_rr>max(lateralLoad))
                        slipRearRight = max(slipAngle);
                         else
                            slipRearRight = min(slipAngle);
                         end
                    end
                    beta_RR(counter+1) = slipRearRight*pi/180;

                psi(counter+1) = psi(counter)+d_psi(counter);
                x_CG(counter+1) = x_CG(counter)+dx_CG*cos(psi(counter+1))-dy_CG*sin(psi(counter+1));
                y_CG(counter+1) = y_CG(counter)+dx_CG*sin(psi(counter+1))+dy_CG*cos(psi(counter+1));

                x_FR(counter+1) = x_CG(counter+1)+sqrt(a^2+(T_front/2)^2)*cos(psi(counter+1)-atan((T_front/2)/a));
                y_FR(counter+1) = y_CG(counter+1)+sqrt(a^2+(T_front/2)^2)*sin(psi(counter+1)-atan((T_front/2)/a));

                % x_RR(counter+1) = x_FR(counter+1)-L*cos(psi(counter+1));
                % y_RR(counter+1) = y_FR(counter+1)-L*sin(psi(counter+1));

                x_RR(counter+1) = x_CG(counter+1)-b*cos(psi(counter+1))+(T_rear/2)*sin(psi(counter+1));
                y_RR(counter+1) = y_CG(counter+1)-b*sin(psi(counter+1))-(T_rear/2)*cos(psi(counter+1));

                x_FL(counter+1) = x_FR(counter+1)-T_front*sin(psi(counter+1));
                y_FL(counter+1) = y_FR(counter+1)+T_front*cos(psi(counter+1));

                x_RL(counter+1) = x_RR(counter+1)-T_rear*sin(psi(counter+1));
                y_RL(counter+1) = y_RR(counter+1)+T_rear*cos(psi(counter+1));
            
            else
                    state = 'straight';
                    state_Num = 0;

                    sigma_FL(counter)=0;
                    sigma_FR(counter)=0;
                    sigma_RL(counter)=0;
                    sigma_RR(counter)=0;

                    x_CC(counter) = 0;
                    y_CC(counter) = 0;

                    x_FC(counter) = x_CG(counter);
                    y_FC(counter) = y_CG(counter);

                    sigma_CG(counter) = 0;

                    R_CG = 0;
                    v_tan(counter) = v;                      
                    omega(counter) = 0;
                    d_psi(counter) = 0;
                    d_S = v_tan(counter)*dt;
                    dx_CG = d_S*cos(psi(counter));
                    dy_CG = d_S*sin(psi(counter));

                    R_F = 0;
                    R_R = 0;
                    v_frontLeft = v_tan(counter);
                    v_frontRight = v_tan(counter);
                    v_rearLeft = v_tan(counter);
                    v_rearRight = v_tan(counter);

                    beta_FL(counter+1) = 0;
                    beta_FR(counter+1) = 0;
                    beta_RL(counter+1) = 0;
                    beta_RR(counter+1) = 0;

                    psi(counter+1) = psi(counter);
                    x_CG(counter+1) = x_CG(counter)+dx_CG;
                    y_CG(counter+1) = y_CG(counter)+dy_CG;

                    x_FR(counter+1) = x_CG(counter+1)+sqrt(a^2+(T_front/2)^2)*cos(psi(counter+1)-atan((T_front/2)/a));
                    y_FR(counter+1) = y_CG(counter+1)+sqrt(a^2+(T_front/2)^2)*sin(psi(counter+1)-atan((T_front/2)/a));

                    x_RR(counter+1) = x_CG(counter+1)-b*cos(psi(counter+1))+(T_rear/2)*sin(psi(counter+1));
                    y_RR(counter+1) = y_CG(counter+1)-b*sin(psi(counter+1))-(T_rear/2)*cos(psi(counter+1));

                    x_FL(counter+1) = x_FR(counter+1)-T_front*sin(psi(counter+1));
                    y_FL(counter+1) = y_FR(counter+1)+T_front*cos(psi(counter+1));

                    x_RL(counter+1) = x_RR(counter+1)-T_rear*sin(psi(counter+1));
                    y_RL(counter+1) = y_RR(counter+1)+T_rear*cos(psi(counter+1));
            end
        end

        if(R_CG~=0)
            acp_CG(counter) = v_tan(counter)^2/R_CG;
        else
            acp_CG(counter) = 0;
        end
        acp_psi(counter) = sigma_CG(counter)+pi/2*(-sign(delta(counter)));

        %dm_lateral(counter) = (mass*(acp_CG(counter)/g)*sin(acp_psi(counter)))/2;
        %dm_longitudinal(counter) = (mass*(acp_CG(counter)/g)*abs(cos(acp_psi(counter))))/2;
        
        %dm_lateral_front(counter) = (-sign(delta(counter))*mass*acp_CG(counter)*abs(sin(acp_psi(counter)))*CG_height/T_front)/g;
        %dm_lateral_rear(counter) = (-sign(delta(counter))*mass*acp_CG(counter)*abs(sin(acp_psi(counter)))*CG_height/T_rear)/g;
        %dm_longitudinal_front(counter) = (mass*acp_CG(counter)*cos(acp_psi(counter))*CG_height/a)/g;
        %dm_longitudinal_rear(counter) = (-mass*acp_CG(counter)*cos(acp_psi(counter))*CG_height/b)/g;

        load_direction = -sign(delta(counter));
        arb_front = 1;
        arb_rear = 1;

        dm_lateral_front(counter) = (acp_CG(counter)*abs(sin(acp_psi(counter)))*CG_height/T_front)/g;
        dm_lateral_front(counter) = dm_lateral_front(counter)*load_direction;
        if(dm_lateral_front(counter)>0.5)
            dm_lateral_front(counter)=0.5;
        else if(dm_lateral_front(counter)<-0.5)
                dm_lateral_front(counter)=-0.5;
            end
        end

        dm_lateral_rear(counter)  = (acp_CG(counter)*abs(sin(acp_psi(counter)))*CG_height/T_rear)/g;
        dm_lateral_rear(counter) = dm_lateral_rear(counter)*load_direction;
        if(dm_lateral_rear(counter)>0.5)
            dm_lateral_rear(counter)=0.5;
        else if(dm_lateral_rear(counter)<-0.5)
                dm_lateral_rear(counter)=-0.5;
            end
        end

        dm_longitudinal_front(counter) = b/L*(-acp_CG(counter)*cos(acp_psi(counter))*CG_height/L)/g;
        dm_longitudinal_rear(counter) = a/L*(acp_CG(counter)*cos(acp_psi(counter))*CG_height/L)/g;

        % dm_longitudinal_front(counter) = (acp_CG(counter)*abs(cos(acp_psi(counter)+pi/2))*CG_height/L)/g;        
        % if(dm_longitudinal_front(counter)>0.5)
        %     dm_longitudinal_front(counter)=0.5;
        % else if(dm_longitudinal_front(counter)<-0.5)
        %         dm_longitudinal_front(counter)=-0.5;
        %     end
        % end
        % dm_longitudinal_rear(counter) = -dm_longitudinal_front(counter);

        m1(counter+1) = (0.5+dm_lateral_front(counter))*arb_front*(m1_s+m2_s)+mass*dm_longitudinal_front(counter);
        m2(counter+1) = (0.5-dm_lateral_front(counter))*arb_front*(m1_s+m2_s)+mass*dm_longitudinal_front(counter);
        m3(counter+1) = (0.5+dm_lateral_rear(counter))*arb_rear*(m3_s+m4_s)+mass*dm_longitudinal_rear(counter);
        m4(counter+1) = (0.5-dm_lateral_rear(counter))*arb_rear*(m3_s+m4_s)+mass*dm_longitudinal_rear(counter);

        % da li i lateral treba ovde deliti sa dva o.O
        % m1(counter+1) = (0.5+dm_lateral_front(counter))*arb_front*(m1_s+m2_s)+dm_longitudinal_front(counter)*m1_s;
        % m2(counter+1) = (0.5-dm_lateral_front(counter))*arb_front*(m1_s+m2_s)+dm_longitudinal_front(counter)*m2_s;
        % m3(counter+1) = (0.5+dm_lateral_rear(counter))*arb_rear*(m3_s+m4_s)+dm_longitudinal_rear(counter)*m3_s;
        % m4(counter+1) = (0.5-dm_lateral_rear(counter))*arb_rear*(m3_s+m4_s)+dm_longitudinal_rear(counter)*m4_s;

        if(m1(counter+1)<1)
            m1(counter+1) = 1;
        end
        if(m2(counter+1)<1)
            m2(counter+1) = 1;
        end
        if(m3(counter+1)<1)
            m3(counter+1) = 1;
        end
        if(m4(counter+1)<1)
            m4(counter+1) = 1;
        end

        if(m1(counter+1)>mass/2-1)
            m1(counter+1) = mass/2-1;
        end
        if(m2(counter+1)>mass/2-1)
            m2(counter+1) = mass/2-1;
        end
        if(m3(counter+1)>mass/2-1)
            m3(counter+1) = mass/2-1;
        end
        if(m4(counter+1)>mass/2-1)
            m4(counter+1) = mass/2-1;
        end

    sigma_CG = [sigma_CG sigma_CG(end)];
    k_F = [k_F k_F(end)];
    k_R = [k_R k_R(end)];
    
    x_CC = [0 x_CC];
    y_CC = [0 y_CC];
    d_psi = [0 d_psi];
    
    x = [x_FL' x_FR' x_RL' x_RR' x_CG' x_CC'];
    y = [y_FL' y_FR' y_RL' y_RR' y_CG' y_CC'];
    beta = [beta_FL' beta_FR' beta_RL' beta_RR'];
  
    acp_CG = [0 acp_CG];    acp_psi = [0 acp_psi];
    dm_longitudinal = [dm_longitudinal_front dm_longitudinal_rear];
    dm_lateral = [dm_lateral_front dm_lateral_rear];

    masses = [m1' m2' m3' m4'];
    
    if(k_R(end)>0)
        dt = 0;
    end

    wheel_speeds(2,:) = [v_frontLeft v_frontRight v_rearLeft v_rearRight];

    output = [x y psi' d_psi' beta sigma_CG' [0 dt]' masses acp_CG' acp_psi' dm_longitudinal' dm_lateral' wheel_speeds];
end