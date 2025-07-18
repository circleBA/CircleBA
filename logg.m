% 파일 경로
log_file = 'cond_rank_log.txt';

% 데이터 읽기 (Condition number, Rank)
data = readmatrix(log_file);

% 분리
cond_list = data(:,1);
rank_list = data(:,2);

% Condition number 시각화
figure;
plot(cond_list, '-o');
xlabel('Frame Index'); ylabel('Condition Number');
title('Hessian Condition Number per Frame');
set(gca, 'YScale', 'log'); grid on;

% Rank 시각화
figure;
plot(rank_list, 's-');
xlabel('Frame Index'); ylabel('Rank of Hessian');
title('Hessian Rank per Frame');
ylim([0 6.5]); yticks(0:6); grid on;

% 조건수 분포 히스토그램 (log scale)
figure;
histogram(log10(cond_list), 30);
xlabel('log10(Condition Number)'); ylabel('Frame Count');
title('Distribution of Condition Number');
