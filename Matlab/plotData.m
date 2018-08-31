function done = plotData(data,vehicle_Params,tire_Params,test)
	if(length(test)==3)
		if(test=='DLC')
			x = data(:,1:6);
			y = data(:,7:12);
			m = data(:,13:21);
			beta = data(:,22:25);
			wheel_speeds = data(:,26:29);

			psi = data(:,30);
			d_psi = data(:,31);
			acp_psi = data(:,32);
			acp_CG = data(:,33);
			sigma_CG = data(:,34);
			steer = data(:,35);
			time = data(:,36);

			orientationVector  = data(:,37);
			timeVector = data(:,38);

		 	x_FL = x(:,1);     y_FL = y(:,1); 
		    x_FR = x(:,2);     y_FR = y(:,2);
		    x_RL = x(:,3);     y_RL = y(:,3);
		    x_RR = x(:,4);     y_RR = y(:,4);
		    x_CG = x(:,5);     y_CG = y(:,5);
		    x_CC = x(:,6);     y_CC = y(:,6);

		    beta_FL = beta(:,1); 
		    beta_FR = beta(:,2);
		    beta_RL = beta(:,3); 
		    beta_RR = beta(:,4); 

		    m1 = m(:,1);       m2 = m(:,2);
		    m3 = m(:,3);       m4 = m(:,4);
		    mass = m(:,5);
		    dm_lateral = m(:,6:7);
		    dm_longitudinal = m(:,8:9);
		    
		    FL_speed = wheel_speeds(:,1);
		    FR_speed = wheel_speeds(:,2);
		    RL_speed = wheel_speeds(:,3);
		    RR_speed = wheel_speeds(:,4);

		    L = vehicle_Params(1);
		    T_front = vehicle_Params(2);
		    T_rear = vehicle_Params(3);
		    CG_height = vehicle_Params(4);
		    a = vehicle_Params(5);
		    b = vehicle_Params(6);
		    g = vehicle_Params(7);
		    speed = vehicle_Params(8);
		    dt = vehicle_Params(9);

		    C_alpha = tire_Params(1);
		    WHEEL_DIAMETER = tire_Params(2);

		    load('tire_Avon.mat');

			sliped = -tire_Avon(:,2);
			lateralLoaded = tire_Avon(:,3).*1e3;

			slipAngle = -12:1e-3:12;
			lateralLoad = interp1(sliped,lateralLoaded,slipAngle,'pchip');


			[trajectory, cones] = doubleLaneChange;

			cones_X = cones(:,1);
			cones_Y_left = cones(:,2);
			cones_Y_right = cones(:,3);
			trajectory_x = trajectory(:,1);
			trajectory_y = trajectory(:,2);

			tg = uitabgroup;
			fig1 = figure(1);
			fig1.Name = 'Vehicle model';
			fig1.Position = [10 55 1342 623];
			fig1.NumberTitle = 'off';

			% fig1 = figure;
			% % fig1.Position = [56 54 1221 617];
			% fig1.Position = [5 63 1221 617];
			tab1 = uitab(tg);
			axes('Parent',tab1);
			tab1.Title = 'Main';

			subplot(4,4,[1 2 5 6 9 10 13 14]);
			plot3(x_CG,y_CG,time,'k',x_FL,y_FL,time,'c',x_FR,y_FR,time,'r',x_RL,y_RL,time,'g',x_RR,y_RR,time,'b');
			hold all
			% cones_time_vector = linspace(0,max(time),length(cones_X));
			plot(cones_X(1:3),cones_Y_left(1:3),'k',...
			      cones_X(4:6),cones_Y_left(4:6),'k',...
			      cones_X(7:9),cones_Y_left(7:9),'k',...
			      cones_X(1:3),cones_Y_right(1:3),'k',...
			      cones_X(4:6),cones_Y_right(4:6),'k',...
			      cones_X(7:9),cones_Y_right(7:9),'k');
			hold all
			plot(cones_X,cones_Y_left,'gO',cones_X,cones_Y_right,'gO',...
			                              'Color',[1 .5 0],'LineWidth',4);
			grid
			title('Trajectory');
			xlabel('x [m]');
			ylabel('y [m]');
			view([0 0 1])
			axis equal
			set(gcf,'Color',[0.85,0.85,0.85]);
			set(gca,'Color',[0.7 0.7 0.7]);

			subplot(4,4,3)
			plot(time,sigma_CG.*(180/pi));
			grid
			xlim([0 max(time)]);
			title('CG speed vector angle');
			xlabel('time [s]');
			ylabel(['\sigma_{CG} [ ',char(176),']']);

			subplot(4,4,7)
			plot(time,d_psi./dt)
			grid
			xlim([0 max(time)]);
			title('Angular speed');
			xlabel('time [s]');
			ylabel('\omega [rad/s]');

			subplot(4,4,4)
			plot(time,d_psi.*180/pi)
			grid
			title('\Delta\Psi');
			xlim([0 max(time)]);

			subplot(4,4,8)
			plot(time,steer.*(135/30));
			grid
			xlabel('time[s]');
			ylabel(['\delta [ ',char(176),']']);
			title('Steering angle');
			xlim([0 max(time)]);

			subplot(4,4,[11 12 15 16]);
			plot(time,m1,'c',time,m2,'r',time,m3,'g',time,m4,'b');
			grid
			xlabel('time [s]');
			ylabel('mass[kg]');
			l = legend('Front_{left}','Front_{right}','Rear_{left}','Rear_{right}','Location','SouthWest');
			l.Position = [0.9141 0.1096 0.0781 0.1618];
			title('Load transfer');
			xlim([0 max(time)]);
			ylim([0 0.6*max(mass)]);

			% fig2 = figure;
			% fig2.Position = [137 46 1221 617];
			tab2 = uitab(tg);
			axes('Parent',tab2);
			tab2.Title = 'Telemetry';
			subplot(231)
			plot(x_CG,y_CG,'k',x_FL,y_FL,'c',x_FR,y_FR,'r',x_RL,y_RL,'g',x_RR,y_RR,'b');
			grid
			axis equal
			title('Trajectory');
			xlabel('X_{axis} [m]');
			ylabel('Y_{axis} [m]');

			subplot(232)
			psi_calc = mod(psi,2*pi).*180/pi;
			psi_calc(psi_calc>180) = psi_calc(psi_calc>180)-360;
			plot(time,psi_calc)
			hold all
			plot(time,orientationVector,'k');
			grid
			title('Orientation');
			xlabel('time [s]');
			ylabel(['\Psi [ ',char(176),']']);
			xlim([0 max(time)]);

			subplot(234)
			plot(time,d_psi.*180/pi)
			grid
			title('\Delta\Psi');
			xlabel('time [s]');
			ylabel(['\Delta\Psi [ ', char(176),']']);
			xlim([0 max(time)]);

			subplot(235)
			plot(time,beta_FL.*180/pi,'c',time,beta_FR.*180/pi,'r');
			hold all
			plot(time,beta_RL.*180/pi,'g',time,beta_RR.*180/pi,'b');
			grid
			title('Slip angle');
			xlabel('time [s]');
			ylabel(['\beta [ ', char(176),']']);
			l = legend('Front_{left}','Front_{right}','Rear_{left}','Rear_{right}','Location','NorthWest');
			% l.Position = [0.5620 0.4581 0.0636 0.0641];
			l.Position = [0.5998 0.3878 0.0778 0.1540];
			xlim([0 max(time)]);

			subplot(233)
			plot(time,x_CC,[time(1) time(end)], [b b],'r');
			grid
			title('Curvature center X_{car}')
			xlabel('time [s]');
			ylabel('Rx_C [m]');
			xlim([0 max(time)]);

			subplot(236)
			plot(time,y_CC);
			grid
			title('Curvature center Y_{car}')
			xlabel('time [s]');
			ylabel('Ry_C [m]');
			% if(median(y_CC)~=0)
			%     ylim([-10*abs(median(y_CC)) 10*abs(median(y_CC))]);
			% else
			%     ylim([-5 5]);
			% end
			xlim([0 max(time)]);

			% fig3 = figure(3);
			% fig3.Position = [1396 81 906 618];
			% % fig3.Position = [75 56 906 618];
			tab3 = uitab(tg);
			axes('Parent',tab3);
			tab3.Title = 'Load transfer';
			subplot(2,4,1)
			plot(time,acp_CG./g);
			grid
			xlabel('time [s]');
			ylabel('Lateral [G]');
			title('A_{cp} CG');
			xlim([0 max(time)]);

			subplot(2,4,2)
			plot(-acp_psi.*180/pi,time);
			grid
			xlabel(['Longitudinal[ ', char(176), ']']);
			ylabel('time [s]');
			title('A_{cp} \Psi');
			ylim([0 max(time)]);

			subplot(2,4,[5 6])
			plot(time,dm_longitudinal);
			grid
			xlabel('time [s]');
			ylabel('load [%]');
			title('\Deltam_{longitudinal}');
			xlim([0 max(time)]);
			legend('Front axle','Rear axle');

			subplot(2,4,[3 4 7 8])
			plot(dm_lateral.*100,time);
			grid
			xlabel('load [%]');
			ylabel('time [s]');
			title('\Deltam_{lateral}');
			ylim([0 max(time)]);
			legend('Front axle','Rear axle');

			% fig4 = figure(4);
			% fig4.Position = [48 87 1269 579];
			% % subplot(211)
			tab4 = uitab(tg);
			axes('Parent',tab4);
			subplot(3,4,[1:8])
			tab4.Title = 'ISO DoubleLaneChange';
			plot(x_FL,y_FL,'c',x_FR,y_FR,'r',x_RL,y_RL,'g',x_RR,y_RR,'b','Linewidth',2.5);
			hold all
			plot(x_CG,y_CG,'k','Linewidth',2.5)
			hold all
			plot(cones_X,cones_Y_left,'gO',cones_X,cones_Y_right,'gO',...
			                              'Color',[1 .5 0],'LineWidth',4);
			hold all
			plot(cones_X(1:3),cones_Y_left(1:3),'k',...
			     cones_X(4:6),cones_Y_left(4:6),'k',...
			     cones_X(7:9),cones_Y_left(7:9),'k',...
			     cones_X(1:3),cones_Y_right(1:3),'k',...
			     cones_X(4:6),cones_Y_right(4:6),'k',...
			     cones_X(7:9),cones_Y_right(7:9),'k');
			hold all
			% plot([0 11 27 39 51 65],[0 0 3 3 0 0],'c','Linewidth',2);
			grid
			axis([-5 65 -5 6]);
			if(max(max(x(:,1:4)))>65)
				xlim([min(min(x(:,1:4)))-5 max(max(x(:,1:4)))+5]);
			end
			if(max(max(y(:,1:4)))>65)
				ylim([min(min(y(:,1:4)))-5 max(max(y(:,1:4)))+5]);
			end
			xlabel('X_{axis} [m]');
			ylabel('Y_{axis} [m]');
			title(['ISO - Double Lane Change ', num2str(speed),'km/h']);
			legend('Front Left','Front Right','Rear Left', 'Rear Right','C/G',...
			       'Cones','Location','South');
			set(gcf,'Color',[0.85,0.85,0.85]);
			set(gca,'Color',[0.7 0.7 0.7]);
			lateral_G = acp_CG./g;
			max_G_txt = ['\leftarrow    G_{lateral} = ',num2str(max(lateral_G)),'G'];
			maxG_xLoc = find(lateral_G==max(lateral_G));
			maxG_yLoc = find(lateral_G==max(lateral_G));
			text(x_RR(maxG_xLoc([1 end])),...
			     y_RR(maxG_yLoc([1 end])),max_G_txt)
			annotation('textbox',...
			            [0.8 0.73 0.10 0.10],...
			            'String',{' 70 km/h'},...
			            'FontSize',14,...
			            'FontName','Arial',...
			            'LineWidth',2,...
			            'BackgroundColor',[0.9  0.9 0.9],...
			            'Color',[0 0 0]);

			% fig5 = figure(5);
			% fig5.Position = [1858 528 787 398];
			% % fig5.Position = [560 272 787 398];
			subplot(3,4,9)
			plot(time,FL_speed);
			grid
			xlim([0 max(time)]);
			title('Front left');
			xlabel('time [s]');
			ylabel('\omega [rad/s]')
			subplot(3,4,10)
			plot(time,RL_speed);
			grid
			xlim([0 max(time)]);
			title('Rear left');
			xlabel('time [s]');
			ylabel('\omega [rad/s]')
			subplot(3,4,11)
			plot(time,RR_speed);
			grid
			xlim([0 max(time)]);
			title('Rear rigth');
			xlabel('time [s]');
			ylabel('\omega [rad/s]')
			subplot(3,4,12)
			plot(time,FR_speed);
			grid
			xlim([0 max(time)]);
			title('Front right');
			xlabel('time [s]');
			ylabel('\omega [rad/s]')

			tab5 = uitab(tg);
			axes('Parent',tab5);
			tab5.Title = 'Car & tires';
			subplot(2,1,1)
			plot(slipAngle,lateralLoad,sliped,lateralLoaded,'r','Linewidth',1.4);
			grid
			ylabel('Lateral force [kN]');
			xlabel(['Slip angle [ ',char(176), ']']);
			title('AVON - Slip angle diagram');
			legend('Approx','Look-up table','Location','SouthEast');

			subplot(2,1,2)
			plot(time,sqrt((x_FL-x_FR).^2+(y_FL-y_FR).^2),'Linewidth',1.6)
			hold all
			plot(time,sqrt((x_RL-x_RR).^2+(y_RL-y_RR).^2),'Linewidth',1.6)
			hold all
			plot(time,sqrt((x_FR-x_RR).^2+(y_FR-y_RR).^2),'Linewidth',1.6)
			hold all
			plot(time,sqrt((x_FL-x_RL).^2+(y_FL-y_RL).^2),'Linewidth',1.6)
			grid
			xlim([0 max(time)]);
			title('Car measures');
			xlabel('time[s]');
			ylabel('Distance [m]');
			legend('Track_{front}','Track_{rear}',...
			       'Wheelbase_{left}','Wheelbase_{right}',...
			       'Location','East');


			tg.SelectedTab = tab4;
			done = 1;

		else if(test=='etc')
				x = data(:,1:6);
				y = data(:,7:12);
				m = data(:,13:21);
				beta = data(:,22:25);
				wheel_speeds = data(:,26:29);

				psi = data(:,30);
				d_psi = data(:,31);
				acp_psi = data(:,32);
				acp_CG = data(:,33);
				sigma_CG = data(:,34);
				steer = data(:,35);
				time = data(:,36);

				orientationVector  = data(:,37);
				timeVector = data(:,38);

			 	x_FL = x(:,1);     y_FL = y(:,1); 
			    x_FR = x(:,2);     y_FR = y(:,2);
			    x_RL = x(:,3);     y_RL = y(:,3);
			    x_RR = x(:,4);     y_RR = y(:,4);
			    x_CG = x(:,5);     y_CG = y(:,5);
			    x_CC = x(:,6);     y_CC = y(:,6);

			    beta_FL = beta(:,1); 
			    beta_FR = beta(:,2);
			    beta_RL = beta(:,3); 
			    beta_RR = beta(:,4); 

			    m1 = m(:,1);       m2 = m(:,2);
			    m3 = m(:,3);       m4 = m(:,4);
			    mass = m(:,5);
			    dm_lateral = m(:,6:7);
			    dm_longitudinal = m(:,8:9);
			    
			    FL_speed = wheel_speeds(:,1);
			    FR_speed = wheel_speeds(:,2);
			    RL_speed = wheel_speeds(:,3);
			    RR_speed = wheel_speeds(:,4);

			    L = vehicle_Params(1);
			    T_front = vehicle_Params(2);
			    T_rear = vehicle_Params(3);
			    CG_height = vehicle_Params(4);
			    a = vehicle_Params(5);
			    b = vehicle_Params(6);
			    g = vehicle_Params(7);
			    speed = vehicle_Params(8);
			    dt = vehicle_Params(9);

			    C_alpha = tire_Params(1);
			    WHEEL_DIAMETER = tire_Params(2);

			    load('tire_Avon.mat');

				sliped = -tire_Avon(:,2);
				lateralLoaded = tire_Avon(:,3).*1e3;

				slipAngle = -12:1e-3:12;
				lateralLoad = interp1(sliped,lateralLoaded,slipAngle,'pchip');

				tg = uitabgroup;
				fig1 = figure(1);
				fig1.Name = 'Vehicle model';
				fig1.Position = [10 55 1342 623];
				fig1.NumberTitle = 'off';

				% fig1 = figure;
				% % fig1.Position = [56 54 1221 617];
				% fig1.Position = [5 63 1221 617];
				tab1 = uitab(tg);
				axes('Parent',tab1);
				tab1.Title = 'Main';

				subplot(4,4,[1 2 5 6 9 10 13 14]);
				plot3(x_CG,y_CG,time,'k',x_FL,y_FL,time,'c',x_FR,y_FR,time,'r',x_RL,y_RL,time,'g',x_RR,y_RR,time,'b');
				hold all
				grid
				
				title('Trajectory');
				xlabel('x [m]');
				ylabel('y [m]');
				view([0 0 1])
				axis equal
				set(gcf,'Color',[0.85,0.85,0.85]);
				set(gca,'Color',[0.7 0.7 0.7]);

				subplot(4,4,3)
				plot(time,sigma_CG.*(180/pi));
				grid
				xlim([0 max(time)]);
				title('CG speed vector angle');
				xlabel('time [s]');
				ylabel(['\sigma_{CG} [ ',char(176),']']);

				subplot(4,4,7)
				plot(time,d_psi./dt)
				grid
				xlim([0 max(time)]);
				title('Angular speed');
				xlabel('time [s]');
				ylabel('\omega [rad/s]');

				subplot(4,4,4)
				plot(time,d_psi.*180/pi)
				grid
				title('\Delta\Psi');
				xlim([0 max(time)]);

				subplot(4,4,8)
				plot(time,steer.*(135/30));
				grid
				xlabel('time[s]');
				ylabel(['\delta [ ',char(176),']']);
				title('Steering angle');
				xlim([0 max(time)]);

				subplot(4,4,[11 12 15 16]);
				plot(time,m1,'c',time,m2,'r',time,m3,'g',time,m4,'b');
				grid
				xlabel('time [s]');
				ylabel('mass[kg]');
				l = legend('Front_{left}','Front_{right}','Rear_{left}','Rear_{right}','Location','SouthWest');
				l.Position = [0.9141 0.1096 0.0781 0.1618];
				title('Load transfer');
				xlim([0 max(time)]);
				ylim([0 0.6*max(mass)]);

				% fig2 = figure;
				% fig2.Position = [137 46 1221 617];
				tab2 = uitab(tg);
				axes('Parent',tab2);
				tab2.Title = 'Telemetry';
				subplot(231)
				plot(x_CG,y_CG,'k',x_FL,y_FL,'c',x_FR,y_FR,'r',x_RL,y_RL,'g',x_RR,y_RR,'b');
				grid
				axis equal
				title('Trajectory');
				xlabel('X_{axis} [m]');
				ylabel('Y_{axis} [m]');

				subplot(232)
				psi_calc = mod(psi,2*pi).*180/pi;
				psi_calc(psi_calc>180) = psi_calc(psi_calc>180)-360;
				plot(time,psi_calc)
				hold all
				plot(time,orientationVector,'k');
				grid
				title('Orientation');
				xlabel('time [s]');
				ylabel(['\Psi [ ',char(176),']']);
				xlim([0 max(time)]);

				subplot(234)
				plot(time,d_psi.*180/pi)
				grid
				title('\Delta\Psi');
				xlabel('time [s]');
				ylabel(['\Delta\Psi [ ', char(176),']']);
				xlim([0 max(time)]);

				subplot(235)
				plot(time,beta_FL.*180/pi,'c',time,beta_FR.*180/pi,'r');
				hold all
				plot(time,beta_RL.*180/pi,'g',time,beta_RR.*180/pi,'b');
				grid
				title('Slip angle');
				xlabel('time [s]');
				ylabel(['\beta [ ', char(176),']']);
				l = legend('Front_{left}','Front_{right}','Rear_{left}','Rear_{right}','Location','NorthWest');
				% l.Position = [0.5620 0.4581 0.0636 0.0641];
				l.Position = [0.5998 0.3878 0.0778 0.1540];
				xlim([0 max(time)]);

				subplot(233)
				plot(time,x_CC,[time(1) time(end)], [b b],'r');
				grid
				title('Curvatore center X_{car}')
				xlabel('time [s]');
				ylabel('Rx_C [m]');
				xlim([0 max(time)]);

				subplot(236)
				plot(time,y_CC);
				grid
				title('Curvatore center Y_{car}')
				xlabel('time [s]');
				ylabel('Ry_C [m]');
				% if(median(y_CC)~=0)
				%     ylim([-10*abs(median(y_CC)) 10*abs(median(y_CC))]);
				% else
				%     ylim([-5 5]);
				% end
				xlim([0 max(time)]);

				% fig3 = figure(3);
				% fig3.Position = [1396 81 906 618];
				% % fig3.Position = [75 56 906 618];
				tab3 = uitab(tg);
				axes('Parent',tab3);
				tab3.Title = 'Load transfer';
				subplot(2,4,1)
				plot(time,acp_CG./g);
				grid
				xlabel('time [s]');
				ylabel('Lateral [G]');
				title('A_{cp} CG');
				xlim([0 max(time)]);

				subplot(2,4,2)
				plot(-acp_psi.*180/pi,time);
				grid
				xlabel(['Longitudinal[ ', char(176), ']']);
				ylabel('time [s]');
				title('A_{cp} \Psi');
				ylim([0 max(time)]);

				subplot(2,4,[5 6])
				plot(time,dm_longitudinal);
				grid
				xlabel('time [s]');
				ylabel('load [%]');
				title('\Deltam_{longitudinal}');
				xlim([0 max(time)]);
				legend('Front axle','Rear axle');

				subplot(2,4,[3 4 7 8])
				plot(dm_lateral.*100,time);
				grid
				xlabel('load [%]');
				ylabel('time [s]');
				title('\Deltam_{lateral}');
				ylim([0 max(time)]);
				legend('Front axle','Rear axle');

				% fig4 = figure(4);
				% fig4.Position = [48 87 1269 579];
				% % subplot(211)
				tab4 = uitab(tg);
				axes('Parent',tab4);
				subplot(3,4,[1:8])
				tab4.Title = 'User defined free test';
				plot(x_FL,y_FL,'c',x_FR,y_FR,'r',x_RL,y_RL,'g',x_RR,y_RR,'b','Linewidth',2.5);
				hold all
				plot(x_CG,y_CG,'k','Linewidth',2.5)
				hold all
				grid
				if(max(max(x(:,1:4)))>65)
					xlim([min(min(x(:,1:4)))-5 max(max(x(:,1:4)))+5]);
				end
				if(max(max(y(:,1:4)))>65)
					ylim([min(min(y(:,1:4)))-5 max(max(y(:,1:4)))+5]);
				end
				xlabel('X_{axis} [m]');
				ylabel('Y_{axis} [m]');
				title(['User defined free test at ', num2str(speed),'km/h']);
				legend('Front Left','Front Right','Rear Left', 'Rear Right','C/G',...
				       'Location','South');
				set(gcf,'Color',[0.85,0.85,0.85]);
				set(gca,'Color',[0.7 0.7 0.7]);
				lateral_G = acp_CG./g;
				max_G_txt = ['\leftarrow    G_{lateral} = ',num2str(max(lateral_G)),'G'];
				maxG_xLoc = find(lateral_G==max(lateral_G));
				maxG_yLoc = find(lateral_G==max(lateral_G));
				text(x_RR(maxG_xLoc([1 end])),...
				     y_RR(maxG_yLoc([1 end])),max_G_txt)
				annotation('textbox',...
				            [0.8 0.73 0.10 0.10],...
				            'String',{' 70 km/h'},...
				            'FontSize',14,...
				            'FontName','Arial',...
				            'LineWidth',2,...
				            'BackgroundColor',[0.9  0.9 0.9],...
				            'Color',[0 0 0]);

				% fig5 = figure(5);
				% fig5.Position = [1858 528 787 398];
				% % fig5.Position = [560 272 787 398];
				subplot(3,4,9)
				plot(time,FL_speed);
				grid
				xlim([0 max(time)]);
				title('Front left');
				xlabel('time [s]');
				ylabel('\omega [rad/s]')
				subplot(3,4,10)
				plot(time,RL_speed);
				grid
				xlim([0 max(time)]);
				title('Rear left');
				xlabel('time [s]');
				ylabel('\omega [rad/s]')
				subplot(3,4,11)
				plot(time,RR_speed);
				grid
				xlim([0 max(time)]);
				title('Rear rigth');
				xlabel('time [s]');
				ylabel('\omega [rad/s]')
				subplot(3,4,12)
				plot(time,FR_speed);
				grid
				xlim([0 max(time)]);
				title('Front right');
				xlabel('time [s]');
				ylabel('\omega [rad/s]')

				tab5 = uitab(tg);
				axes('Parent',tab5);
				tab5.Title = 'Car & tires';
				subplot(2,1,1)
				plot(slipAngle,lateralLoad,sliped,lateralLoaded,'r','Linewidth',1.4);
				grid
				ylabel('Lateral force [kN]');
				xlabel(['Slip angle [ ',char(176), ']']);
				title('AVON - Slip angle diagram');
				legend('Approx','Look-up table','Location','SouthEast');

				subplot(2,1,2)
				plot(time,sqrt((x_FL-x_FR).^2+(y_FL-y_FR).^2),'Linewidth',1.6)
				hold all
				plot(time,sqrt((x_RL-x_RR).^2+(y_RL-y_RR).^2),'Linewidth',1.6)
				hold all
				plot(time,sqrt((x_FR-x_RR).^2+(y_FR-y_RR).^2),'Linewidth',1.6)
				hold all
				plot(time,sqrt((x_FL-x_RL).^2+(y_FL-y_RL).^2),'Linewidth',1.6)
				grid
				xlim([0 max(time)]);
				title('Car measures');
				xlabel('time[s]');
				ylabel('Distance [m]');
				legend('Track_{front}','Track_{rear}',...
				       'Wheelbase_{left}','Wheelbase_{right}',...
				       'Location','East');


				tg.SelectedTab = tab4;
				done = 1;
			else
				disp('Wrong test input. Set DLC(Double Lane Change) or etc(user defined free test');
			end
		end
	else

			disp('Wrong test input. Set DLC(Double Lane Change) or etc(user defined free test');
	end
end