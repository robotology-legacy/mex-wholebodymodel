function B = rotm2angVelTF(rotm, rtype, sequence)
    switch nargin
        case 3
            if ~strcmp(rtype, 'eul')
                error('rotm2angVelTF: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
            eul = WBM.utilities.tfms.rotm2eul(rotm, sequence);
            B   = WBM.utilities.tfms.eul2angVelTF(eul);
        case 2
            switch rtype
                case 'eul'
                    % use the default ZYX axis sequence ...
                    eul = WBM.utilities.tfms.rotm2eul(rotm);
                    B   = WBM.utilities.tfms.eul2angVelTF(eul);
                case 'quat'
                    quat = WBM.utilities.tfms.rotm2quat(rotm);
                    B    = WBM.utilities.tfms.quat2angVelTF(quat);
                case 'axang'
                    axang = WBM.utilities.tfms.rotm2axang(rotm);
                    B     = WBM.utilities.tfms.axang2angVelTF(axang);
                otherwise
                    error('rotm2angVelTF: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
        otherwise
            error('rotm2angVelTF: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
