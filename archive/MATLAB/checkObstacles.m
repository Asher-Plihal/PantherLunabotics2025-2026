function obs = checkObstacles(x,obs,all_obs)

    for i = 1:size(all_obs,1)
        obs_loc = all_obs{i,1};
        obs_rad = all_obs{i,2};
        dist = norm(x(1:2)-obs_loc);
        if (dist < 1.5) && ~any(cellfun(@(x) isequal(x, obs_loc), obs(:,1)))
            obs(end + 1,:) = {obs_loc, obs_rad};
        end
    end
end