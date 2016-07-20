function pinvDampA = pinvDamped(A,regDamp)
%     [U,S,V] = svd(A*A');
%     for i=1:min(size(S))
%         if S(i,i) <= regDamp
%             S(i,i) = regDamp;
%         end
%     end
%     pinvDampA = A'/(U*S*V');

    pinvDampA = A'/(A*A' + regDamp*eye(size(A,1)));
end