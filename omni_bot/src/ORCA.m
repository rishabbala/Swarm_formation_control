%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%                OPTIMAL RECIPROCAL COLLISION            %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%{
1. Considering for all bots v_opt = v_cur and v_max = 1m/s
2. Check if Velocity Obstacle
3. Take current velocity of bot to be v_cur
4. Compute ORCA for each bot given other as v: (v-(v_cur+0.5u)).n>=0 and
similarly for other bot.
5. Consider all velocities less than v_max
6. Take new velocity as arg_min||(v-v_pref)||
%}


