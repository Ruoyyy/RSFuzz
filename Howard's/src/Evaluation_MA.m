for i =1:2000
    try
        test_MA_fuzzer
    catch
        continue
    end
end