function make_combined_fig1_for_paper()

    imgA = imread('fig1a_env_obstacles_MCmedian.png');
    imgB = imread('fig1b_error_nrx_MCmedian.png');

    % 흰 여백 자르기
    imgA = cropWhiteMargin(imgA);
    imgB = cropWhiteMargin(imgB);

    % 최종 캔버스 크기
    canvasH = 900;
    gap = 35;

    % (a) 환경 그림 크기 조정
    imgA = resizeToHeight(imgA, canvasH);

    % (b) 결과 그림 크기 조정
    imgB = resizeToHeight(imgB, canvasH);

    % 너무 넓으면 결과 그림을 조금 줄임
    maxTotalW = 2600;
    totalW = size(imgA,2) + gap + size(imgB,2);

    if totalW > maxTotalW
        scale = (maxTotalW - size(imgA,2) - gap) / size(imgB,2);
        imgB = imresize(imgB, scale);
    end

    % 높이 맞추기
    H = max(size(imgA,1), size(imgB,1));
    W = size(imgA,2) + gap + size(imgB,2);

    canvas = uint8(255 * ones(H, W, 3));

    yA = floor((H - size(imgA,1))/2) + 1;
    yB = floor((H - size(imgB,1))/2) + 1;

    canvas(yA:yA+size(imgA,1)-1, 1:size(imgA,2), :) = imgA;

    xB = size(imgA,2) + gap + 1;
    canvas(yB:yB+size(imgB,1)-1, xB:xB+size(imgB,2)-1, :) = imgB;

    imwrite(canvas, 'fig1_combined_paper.png');

    figure('Color','w');
    imshow(canvas);
    title('그림 1 최종 합본');

    disp('Saved: fig1_combined_paper.png');

end

function imgOut = cropWhiteMargin(img)
    if size(img,3) == 3
        gray = rgb2gray(img);
    else
        gray = img;
    end

    mask = gray < 250;
    rows = find(any(mask,2));
    cols = find(any(mask,1));

    if isempty(rows) || isempty(cols)
        imgOut = img;
        return;
    end

    pad = 15;
    r1 = max(1, rows(1)-pad);
    r2 = min(size(img,1), rows(end)+pad);
    c1 = max(1, cols(1)-pad);
    c2 = min(size(img,2), cols(end)+pad);

    imgOut = img(r1:r2, c1:c2, :);
end

function imgOut = resizeToHeight(img, targetH)
    scale = targetH / size(img,1);
    imgOut = imresize(img, scale);
end