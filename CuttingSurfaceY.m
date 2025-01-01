function hPoints = CuttingSurfaceY(Points, cen)
% return the half sphere surface interacting with workpiece

ref_y = cen.y;
hPoints = Points;
for i = length(Points):-1:1
    if Points(2,i) > ref_y
        hPoints(:,i) = [];
    end
end