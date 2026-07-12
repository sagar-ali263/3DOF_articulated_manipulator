function checkSingularity(robot,q)

J = geometricJacobian(robot,q,'body3');

Jv = J(4:6,:);

rank_J = rank(Jv);

manipulability = sqrt(abs(det(Jv*Jv')));

fprintf('\n-------------------------------\n');
fprintf('Singularity Check\n');
fprintf('-------------------------------\n');

fprintf('Jacobian Rank = %d\n',rank_J);
fprintf('Manipulability = %.3f\n',manipulability);

threshold = 1e5;

if rank_J < 3
    warning('Robot is in a singular configuration!');
elseif manipulability < threshold
    warning('Robot is close to singularity!');
else
    disp('Configuration is safe.');
end

end