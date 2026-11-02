function save_figure(f, name)
%SAVE_FIGURE  Export a figure to results/<name>.png with a light theme.
    if exist('theme', 'file')
        theme(f, 'light');
    end
    out = fullfile(fileparts(mfilename('fullpath')), 'results');
    if ~exist(out, 'dir'), mkdir(out); end
    exportgraphics(f, fullfile(out, [name '.png']), 'Resolution', 140);
end
