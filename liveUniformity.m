function liveUniformity(cam, varargin)
%LIVEUNIFORMITY  Live false-colour view of scene luminance to judge flat-field
%  illumination uniformity (hot spots / gradients) while positioning the target
%  for OETF/PTC. Loops until the figure window is closed.
%
%    liveUniformity(cam)                       % ~5 fps, green-plane luma, turbo map
%    liveUniformity(cam,'fps',8)               % faster refresh
%    liveUniformity(cam,'mode','gray')         % rgb2gray(demosaic) luma (slower)
%
%  Uses cam.grab() (raw Bayer, fast -- no HDR engine, no exposure settle). The
%  green plane is a clean luminance proxy; the frame is auto-stretched so gradients
%  pop, with a colorbar. The title reports central-ROI CV and a 3x3 tile spread
%  (aim for CV < ~0.05 and spread < ~15% for a good flat field).
    p = inputParser;
    p.addParameter('fps',  5,       @(x)isnumeric(x) && x > 0);
    p.addParameter('mode', 'green', @(s)any(strcmpi(s, {'green','gray'})));
    p.parse(varargin{:});
    period  = 1 / p.Results.fps;
    useGray = strcmpi(p.Results.mode, 'gray');

    fig = figure('Name','Live illumination uniformity','NumberTitle','off','Color','k');
    ax  = axes('Parent', fig);
    im  = imagesc(ax, zeros(2,2));
    axis(ax, 'image', 'off');
    try
        colormap(ax, turbo);                                  % turbo may be absent pre-R2020a
    catch
        colormap(ax, jet);
    end
    colorbar(ax, 'Color', 'w');
    tt = title(ax, 'starting...', 'Color', 'w');

    while isvalid(fig)
        t0 = tic;
        try
            if useGray
                lum = double(rgb2gray(cam.captureRGB()));
            else
                raw = cam.grab();
                lum = double(raw(1:2:end, 2:2:end));          % Gr plane (half-res green)
            end
        catch e
            if isvalid(tt), tt.String = ['capture error: ' e.message]; end
            drawnow limitrate; pause(0.5); continue
        end
        set(im, 'CData', lum);
        lo = min(lum(:)); hi = max(lum(:));
        if hi > lo, set(ax, 'CLim', [lo hi]); end             % auto-stretch -> gradients pop

        [h, w] = size(lum);
        r  = round(min(h, w) * 0.15); cy = round(h/2); cx = round(w/2);
        roi = lum(cy-r:cy+r, cx-r:cx+r);
        cv  = std(roi(:)) / max(mean(roi(:)), 1);
        ty = floor(h/3); tx = floor(w/3); tm = zeros(3,3);
        for iy = 1:3
            for ix = 1:3
                tm(iy,ix) = mean(mean(lum((iy-1)*ty+1:iy*ty, (ix-1)*tx+1:ix*tx)));
            end
        end
        spread = (max(tm(:)) - min(tm(:))) / max(mean(tm(:)), 1);
        if isvalid(tt)
            tt.String = sprintf(['mean %.0f   central CV %.3f   tile spread %.1f%%' ...
                '   (target: CV<0.05, spread<15%%)'], mean(roi(:)), cv, 100*spread);
            if cv < 0.05 && spread < 0.15, tt.Color = [0.4 0.9 0.5]; else, tt.Color = [0.95 0.8 0.4]; end
        end
        drawnow limitrate;
        pause(max(0, period - toc(t0)));
    end
end
