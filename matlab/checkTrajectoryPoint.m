function safe = checkTrajectoryPoint(robot,q)

J = geometricJacobian(robot,q,'body3');
Jv = J(4:6,:);

rankJ = rank(Jv);

manipulability = sqrt(abs(det(Jv*Jv')));

minimumManipulability = 1e6;

safe = true;

if rankJ < 3

    warning('Singular configuration detected.');
    safe = false;

elseif manipulability < minimumManipulability

    warning('Near singular configuration detected.');
    safe = false;

end

end