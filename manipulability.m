function mu = manipulability(J, measure)
switch measure 
    case 'sigmamin'
    mu = min(svd(J));
    case 'detjac'
    mu = det(J);
    case 'invcond'
    mu = min(svd(J)) ./ max(svd(J));
end
end