function wgdop = computeWGDOP(Hpos, R)

if isempty(Hpos) || size(Hpos,1) < 3
    wgdop = nan;
    return;
end

FIM = Hpos' / ensurePD(R) * Hpos;
FIM = ensurePD(FIM);
wgdop = sqrt(trace(inv(FIM)));

end