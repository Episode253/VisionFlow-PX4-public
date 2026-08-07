function validateForm() {
    var valid = true;
    var error_html = "";
    if (document.getElementById('file').value.length == 0) {
        error_html += "Log File, "
        valid = false;
    }

    if (!valid) {
        alert("Missing fields: " + error_html.substring(0, error_html.length - 2));
    }

    console.log('validateForm result:', valid);
    return valid;
}

function doUpload() {
    console.log('doUpload called');
    if (!validateForm()) {
        console.log('Validation failed, aborting upload');
        return;
    }

    var upload_button = $('#upload-button');
    upload_button.text('Uploading...').prop('disabled', true);

    var progress_container = $('#progress-container');
    var progress_bar = $('#progress-bar');
    progress_container.show();
    progress_bar.width('0%').html('0%');

    var form_data = new FormData($("#upload-form")[0]);
    console.log('File selected:', form_data.get('filearg'));

    $.ajax({
        xhr: function () {
            var xhr = new window.XMLHttpRequest();
            xhr.upload.addEventListener("progress", function (evt) {
                if (evt.lengthComputable) {
                    var percent_complete = (evt.loaded / evt.total) * 99;
                    progress_bar.width(percent_complete + '%');
                    progress_bar.html(percent_complete.toFixed(0) + '%');
                }
            }, false);
            return xhr;
        },
        type: 'POST',
        url: '/upload',
        data: form_data,
        cache: false,
        contentType: false,
        processData: false,
        success: function (data) {
            console.log('Upload success:', data);
            var json_response = JSON.parse(data);
            console.log('Redirecting to:', json_response.url);
            window.location.href = json_response.url;
        },
        error: function (data, textStatus, errorThrown) {
            console.error('Error uploading file:', textStatus, errorThrown);
            progress_container.hide();
            $('#upload-failure').show();
            upload_button.text('Upload').prop('disabled', false);
        }
    });
}

$(function() {
    console.log('jQuery loaded, page ready');
    console.log('Upload button found:', $('#upload-button').length > 0);
    console.log('File input found:', $('#file').length > 0);

    // File input change handler
    $('#file').on('change', function() {
        var fileName = $(this).val().split('\\').pop();
        if (fileName) {
            $('#file-name').text(fileName);
            $('#upload-zone').addClass('has-file');
            console.log('File selected:', fileName);
        } else {
            $('#file-name').text('');
            $('#upload-zone').removeClass('has-file');
        }
    });

    $('#upload-button').on('click', function() {
        console.log('Upload button clicked');
        doUpload();
    });
});
