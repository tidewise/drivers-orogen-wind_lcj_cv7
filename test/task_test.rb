# frozen_string_literal: true

using_task_library "wind_lcj_cv7"

describe OroGen.wind_lcj_cv7.Task do
    run_live

    attr_reader :task

    before do
        @task = syskit_deploy(
            OroGen.wind_lcj_cv7.Task.deployed_as("wind_lcj_cv7_Task_test")
        )

        # This complicated setup works around that data readers and writers
        # in Syskit connect themselves only when their target tasks are running
        #
        # We need to be connected before configure
        io_reader_writer_m = Syskit::DataService.new_submodel do
            input_port "in", "/iodrivers_base/RawPacket"
            output_port "out", "/iodrivers_base/RawPacket"
        end

        cmp_m = Syskit::Composition.new_submodel do
            add io_reader_writer_m, as: "reader_writer"
            add OroGen.nmea0183.MarnavTask, as: "marnav"

            reader_writer_child.connect_to marnav_child.io_raw_in_port
            marnav_child.io_raw_out_port.connect_to reader_writer_child
        end
        cmp = syskit_stub_deploy_configure_and_start(
            cmp_m.use("marnav" => @task, "reader_writer" => io_reader_writer_m)
        )
        @io = cmp.reader_writer_child

        @mwv_sentence = make_packet("$WIMWV,214.8,R,0.1,K,A*28\r\n")
        @xdr_sentence = make_packet("$IIXDR,C,19.52,C,TempAir*19\r\n")
    end

    it "processes a MWV message" do
        tic = Time.now
        speed = expect_execution { syskit_write @io.out_port, @mwv_sentence }
                .to { have_one_new_sample task.air_speed_port }
        toc = Time.now

        expected_speed_v = Eigen::Quaternion.from_angle_axis(
            214.8 * Math::PI / 180, Eigen::Vector3.UnitZ
        ) * Eigen::Vector3.new(0.1 / 3.6, 0, 0)
        expected_direction_v = expected_speed_v.normalize
        direction_v = speed.velocity.normalize

        assert_operator tic, :<, speed.time
        assert_operator toc, :>, speed.time
        assert_in_delta (expected_direction_v - direction_v).norm, 0, 1e-3
        assert_in_delta (expected_speed_v - speed.velocity).norm, 0, 1e-3
    end

    it "processes a XDR message" do
        tic = Time.now
        sample = expect_execution { syskit_write @io.out_port, @xdr_sentence }
                 .to { have_one_new_sample task.air_temperature_port }
        toc = Time.now

        assert_operator tic, :<, sample.time
        assert_operator toc, :>, sample.time
        assert_in_delta 19.52 + 273.15, sample.kelvin, 1e-3
    end

    def make_packet(sentence)
        Types.iodrivers_base.RawPacket.new(
            time: Time.now,
            data: sentence.each_byte.to_a
        )
    end
end
