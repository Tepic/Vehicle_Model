function [orientationVector timeVector] = generateOrientation(orientationVector,timeVector,finalOrientation,time_transition,dt,style)
	if(length(style)==4)
		if(mean(style=='step') || mean(style=='ramp'))
			multiplier = round(time_transition/dt);
			addTime = linspace(timeVector(end,1),timeVector(end,1)+multiplier*dt,multiplier+1);

			timeVector = [timeVector; addTime'];
			if(mean(style=='ramp')==1)
				addOrientation = linspace(orientationVector(end,1),finalOrientation,multiplier+1);
			else if(mean(style=='step')==1)
					addOrientation = linspace(finalOrientation,finalOrientation,multiplier+1);
  				 end
			end
			orientationVector = [orientationVector; addOrientation'];
		else

		disp('Wrong input for signal configuration. RAMP or STEP are supported ONLY currently');
		end
	else
		disp('Wrong input for signal configuration. RAMP or STEP are supported ONLY currently');
	end
end