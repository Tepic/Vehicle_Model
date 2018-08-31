function [output,cones] = doubleLaneChange()

	% generate double-lane change cones
	%cones_X = [10 15 20 31 37.5 44 57 63 69]-10;

	cones_X = [0 6 12 25.5 31 36.5 49 55 61]+830*1e-3;
	cones_Y_left = [0.8125 0.8125 0.8125...
                    3.9375 3.9375 3.9375...
                    1.5    1.5    1.5];
	cones_Y_right = [-0.8125 -0.8125 -0.8125...
                      1.8125  1.8125  1.8125...
                     -1.5    -1.5    -1.5];
	cones = [cones_X; cones_Y_left; cones_Y_right]';
%'
	% generate vehicle trajectory
	s_1 = atan(-5:0.05:4.5); %-5.7
	s_1 = s_1+1.45;
	s_1 = s_1-s_1(1);
	% s_2 = -atan(-5.7:0.05:4.5); %neprekidnost funkcije s_1 i s_2
	s_2 = -atan(-4:0.05:4.5);
	s_2 = s_2+1.45;
	s_2 = s_2-s_2(end);
	x_1 = 0:0.05:15;
	x_2 = linspace(15,37.5,length(s_1));
	x_3 = linspace(37.5,63,length(s_2));
	x_4 = 63:0.05:80;
	p_1 = zeros(1,length(x_1));
	p_4 = zeros(1,length(x_4));
	x = [x_1 x_2 x_3 x_4];
	y = [p_1 s_1 s_2 p_4];

	output = [x' y'];
end