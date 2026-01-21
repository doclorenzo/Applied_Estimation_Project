function lm = barcode_to_lm_index(est, barcode)
% Returns lm_index in 1..nL, or [] if not a landmark.

    if isempty(est.barcodes)
        lm = [];
        return;
    end
    
    subject = find(est.barcodes == barcode, 1, 'first');
    if isempty(subject)
        lm = [];
        return;
    end

    % subjects 1..5 are robots, 6.. are landmarks
    if subject <= 5
        lm = []; % robot measurement (ignore)
        return;
    end

    lm = subject - 5;
end
