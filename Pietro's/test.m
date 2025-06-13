for m =1:2000
    try
        hard_simulation_config
        MA_Fuzzing
    catch
        continue
    end
end

