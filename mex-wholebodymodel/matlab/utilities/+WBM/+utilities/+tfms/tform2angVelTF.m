function B = tform2angVelTF(tform, rtype, sequence)
    switch nargin
        case 3
            if ~strcmp(rtype, 'eul')
                error('tform2angVelTF: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
            eul = WBM.utilities.tfms.tform2eul(tform, sequence);
            B   = WBM.utilities.tfms.eul2angVelTF(eul);
        case 2
            switch rtype
                case 'eul'
                    % use the default ZYX axis sequence ...
                    eul = WBM.utilities.tfms.tform2eul(tform);
                    B   = WBM.utilities.tfms.eul2angVelTF(eul);
                case 'quat'
                    quat = WBM.utilities.tfms.tform2quat(tform);
                    B    = WBM.utilities.tfms.quat2angVelTF(quat);
                case 'axang'
                    axang = WBM.utilities.tfms.tform2axang(tform);
                    B     = WBM.utilities.tfms.axang2angVelTF(axang);
                otherwise
                    error('tform2angVelTF: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
        otherwise
            error('tform2angVelTF: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
end
