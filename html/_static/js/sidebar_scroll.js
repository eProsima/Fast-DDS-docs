(function () {
    var KEY = 'fds-sidebar-scroll';

    function sidebar() {
        return document.querySelector('.sidebar-scroll');
    }

    document.addEventListener('DOMContentLoaded', function () {
        var el = sidebar();
        if (!el) return;
        var saved = sessionStorage.getItem(KEY);
        if (saved !== null) {
            el.scrollTop = parseInt(saved, 10);
        }
    });

    window.addEventListener('beforeunload', function () {
        var el = sidebar();
        if (el) {
            sessionStorage.setItem(KEY, el.scrollTop);
        }
    });
})();
