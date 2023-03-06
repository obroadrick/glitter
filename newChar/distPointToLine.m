function d = distPointToLine(point, pointOnLine, direction)
    %a = pointOnLine';% point on the line
    %n = (direction./norm(direction))';% unit vector in direction of line
    %p = point';% point whose distance is being computed
    %d = norm((p-a)-(dot((p-a),n,1)*n));

    %quicker?
    d = norm((point'-pointOnLine')-(dot((point'-pointOnLine'),(direction./norm(direction))',1)*(direction./norm(direction))'));

end