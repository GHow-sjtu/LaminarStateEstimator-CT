function hPoints = CuttingSurfaceX(Points, cen)
% return the half sphere surface interacting with workpiece

ref_x = cen(1);
hPoints = Points;
for i = length(Points):-1:1
    if Points(1,i) < ref_x
        hPoints(:,i) = [];
    end
end
