t = 3;
filename_txt = 'result.txt'; % 文件名
fid = fopen(filename_txt, 'a'); % 打开文件以供写入
fprintf(fid, '%d\n', t); % 将变量的值写入文件
fclose(fid); % 关闭文件
error("!")