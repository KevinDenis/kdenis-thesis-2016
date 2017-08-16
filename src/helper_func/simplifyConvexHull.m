function TS_occ_convex_simple=simplifyConvexHull(TS_occ_convex)

[t_min,idx_t_min] = min(TS_occ_convex(:,1));
[t_max,idx_t_min] = max(TS_occ_convex(:,1));

[s_min,idx_s_min] = min(TS_occ_convex(:,2));
[s_max,idx_s_min] = max(TS_occ_convex(:,2));

Extemas = [t_max s_max;t_min s_min];

TS_occ_new = [TS_occ_convex;Extemas];

kk_convex = convhull(TS_occ_new(:,1),TS_occ_new(:,2),'simplify', true);

TS_occ_convex_simple = TS_occ_new(kk_convex,:);
end
