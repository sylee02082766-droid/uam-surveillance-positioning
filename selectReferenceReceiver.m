function ref_idx_local = selectReferenceReceiver(received_power_detected_dBm)
% selectReferenceReceiver
%
% A1/A2 terminal MLAT simulator용 reference receiver 선택 함수.
%
% 입력:
%   received_power_detected_dBm : 검출된 MLAT 수신기들의 수신전력 [dBm]
%
% 출력:
%   ref_idx_local : 검출된 수신기 배열 내부에서 기준 수신기의 local index
%
% 현재 구현:
%   가장 수신전력이 큰 수신기를 기준 수신기로 선택한다.
%
% 주의:
%   이 함수는 이전 ablation 코드의 applyDefaultAblationOptions,
%   subset selection, WGDOP reference selection 등을 사용하지 않는다.

    if isempty(received_power_detected_dBm)
        ref_idx_local = [];
        return;
    end

    [~, ref_idx_local] = max(received_power_detected_dBm);

end