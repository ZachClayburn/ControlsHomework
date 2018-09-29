from Animations import Animation
from matplotlib.artist import Artist
from typing import List
import matplotlib.animation as mpl_animation


def view_animation(animation: Animation, q_0: List[int], q) -> mpl_animation.FuncAnimation:
    return animation.animate(q_0, q)


def save_movie(animation: Animation, q_0, q, save_file_name: str):
    fig = animation.figure
    ax = animation.axis

    def update_blit(artists_list: List[Artist]):
        fig.canvas.restore_region(bg_cache)
        for artist in artists_list:
            artist.axes.draw_artist(artist)

        ax.figure.canvas.blit(ax.bbox)

    artist_list = animation.get_init_func(q_0)()
    step_func = animation.get_step_func()

    for a in artist_list:
        a.set_animated(True)

    fig.canvas.draw()
    bg_cache = fig.canvas.copy_from_bbox(ax.bbox)

    # movie_writer = mpl_animation.MovieWriter(fps=25, )
    ffmpeg_writer = mpl_animation.writers['ffmpeg']
    movie_writer = ffmpeg_writer(fps=25)

    with movie_writer.saving(fig, save_file_name, 100):
        movie_writer.grab_frame()
        for frame in q:
            artist_list = step_func(frame)
            update_blit(artist_list)
            movie_writer.grab_frame()
