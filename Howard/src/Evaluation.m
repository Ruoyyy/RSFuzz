for i =1:2000
    try
        test_fuzzer
    catch
        continue
    end
end