function B_inv = rotm2angRateTF(rotm, rtype, sequence)
    switch nargin
        case 3
            if ~strcmp(rtype, 'eul')
                error('rotm2angRateTF: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
            eul   = WBM.utilities.rotm2eul(rotm, sequence);
            B_inv = WBM.utilities.eul2angRateTF(eul);
        case 2
            switch rtype
                case 'eul'
                    % use the default ZYX axis sequence ...
                    eul   = WBM.utilities.rotm2eul(rotm);
                    B_inv = WBM.utilities.eul2angRateTF(eul);
                case 'quat'
                    quat  = WBM.utilities.rotm2quat(rotm);
                    B_inv = WBM.utilities.quat2angRateTF(quat);
                case 'axang'
                    axang = WBM.utilities.rotm2axang(rotm);
                    B_inv = WBM.utilities.axang2angRateTF(axang);
                otherwise
                    error('rotm2angRateTF: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
        otherwise
            error('rotm2angRateTF: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
